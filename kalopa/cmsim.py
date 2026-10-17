#
# Copyright 2026, Kalopa Robotics Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Kalopa command module simulator: the CM side of the payload link,
# over TCP.
#
#    python3 -m kalopa.cmsim [--port 5010] [--uplink 60] [--verbose]
#
# It sails a simulated boat round a few waypoints in Galway Bay and
# behaves the way the real command module will towards a payload:
#
#  * SUB/UNSUB/GET for standard NMEA telemetry (RMC GGA HDT ROT RSA MWV RMB)
#    and XDR groups (XDR.IMU, XDR.ENV, XDR.CTRL), max 10 Hz each, with an
#    aggregate character budget for the link speed
#  * SAVE / XMIT / NOTE of tagged values, tag dictionary, self-describing
#    "microSD" log
#  * XMIT values coalesced per tag and sent in periodic simulated Iridium
#    uplinks, charged against a byte token bucket
#  * ACK / NAK replies, heartbeat watchdog
#
import argparse
import math
import re
import socket
import struct
import threading
import time

from . import nmea

TALKER = "PKLP"
MAX_RATE = 10.0
MAX_TAGS = 32
TAG_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,7}$")
HB_TIMEOUT = 30.0

#
# TLV type codes
TLV_PAYLOAD_TAGDEF = 0x60      # body: index (1) + name (8, NUL padded)  -> 10 bytes on wire
TLV_PAYLOAD_VALUE = 0x61       # body: index (1) + float32 LE (4)        ->  6 bytes on wire
TAG_UPLINK_COST = 16           # each uplinked tag carries its definition + its value

WAYPOINTS = [                  # (name, lat, lon) -- a lap of inner Galway Bay
    ("WP1", 53.2200, -9.0500),
    ("WP2", 53.1850, -9.1600),
    ("WP3", 53.1550, -9.0200),
]


def log(msg):
    print("%s cmsim: %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


#
# A crude but plausible boat. Your mileage may vary, etc.
class Boat:
    def __init__(self):
        self.lat, self.lon = 53.1700, -8.9900          # off Galway Bay
        self.heading = 270.0
        self.rot = 0.0                                 # deg/min
        self.sog = 0.0                                 # knots
        self.rudder = 0.0
        self.wing = 0.0
        self.wind_dir, self.wind_speed = 240.0, 12.0   # true, knots
        self.pitch = self.roll = 0.0
        self.batt, self.solar, self.board = 13.2, 0.0, 22.0
        self.wp = 0
        self.prev_wp = ("START", self.lat, self.lon)
        self.t0 = time.time()
        self._lock = threading.Lock()

    def step(self, dt):
        with self._lock:
            t = time.time() - self.t0
            name, wlat, wlon = WAYPOINTS[self.wp]
            rng, brg = self.range_bearing(wlat, wlon)
            if rng < 0.05:                              # arrived: next leg
                self.prev_wp = WAYPOINTS[self.wp]
                self.wp = (self.wp + 1) % len(WAYPOINTS)
            err = (brg - self.heading + 540) % 360 - 180
            self.rudder = max(-30.0, min(30.0, err * 0.6))
            turn = self.rudder * 0.15 * dt * 10         # deg this step
            self.heading = (self.heading + turn) % 360
            self.rot = turn / dt * 60 if dt else 0
            awa = (self.wind_dir - self.heading + 540) % 360 - 180
            self.wing = max(-25.0, min(25.0, awa * 0.25))
            self.sog = max(0.5, 3.0 * abs(math.sin(math.radians(awa))) + 0.3 * math.sin(t / 7))
            dist_nm = self.sog * dt / 3600.0
            self.lat += dist_nm / 60.0 * math.cos(math.radians(self.heading))
            self.lon += dist_nm / 60.0 * math.sin(math.radians(self.heading)) / math.cos(math.radians(self.lat))
            self.wind_dir = 240 + 15 * math.sin(t / 90)
            self.wind_speed = 12 + 3 * math.sin(t / 23)
            self.pitch = 4 * math.sin(t * 1.3)
            self.roll = -awa / 8 + 3 * math.sin(t * 0.9)
            hour = time.localtime().tm_hour + time.localtime().tm_min / 60
            self.solar = max(0.0, 21 * math.sin(math.pi * (hour - 7) / 12)) if 7 <= hour <= 19 else 0.0
            self.batt = 12.4 + (0.9 if self.solar > 13 else 0.2) + 0.05 * math.sin(t / 60)
            self.board = 22 + 6 * (self.solar / 21) + 0.3 * math.sin(t / 40)

    def range_bearing(self, lat, lon):
        dlat = (lat - self.lat) * 60
        dlon = (lon - self.lon) * 60 * math.cos(math.radians(self.lat))
        return math.hypot(dlat, dlon), (math.degrees(math.atan2(dlon, dlat)) + 360) % 360

    # --- sentence generators: each returns a list of complete sentences ---
    def _fix(self):
        now = time.gmtime()
        hms = time.strftime("%H%M%S", now) + ".00"
        lat, ns = nmea.from_degrees(self.lat, True)
        lon, ew = nmea.from_degrees(self.lon, False)
        return now, hms, lat, ns, lon, ew

    def rmc(self):
        now, hms, lat, ns, lon, ew = self._fix()
        return [nmea.build("GPRMC", hms, "A", lat, ns, lon, ew, "%.1f" % self.sog,
                           "%.1f" % self.heading, time.strftime("%d%m%y", now), "", "", "A")]

    def gga(self):
        _, hms, lat, ns, lon, ew = self._fix()
        return [nmea.build("GPGGA", hms, lat, ns, lon, ew, 1, "08", "0.9", "1.5", "M", "55.0", "M", "", "")]

    def hdt(self):
        return [nmea.build("KLHDT", "%.1f" % self.heading, "T")]

    def rot_(self):
        return [nmea.build("KLROT", "%.1f" % self.rot, "A")]

    def rsa(self):
        return [nmea.build("KLRSA", "%.1f" % self.rudder, "A", "", "V")]

    def mwv(self):
        awa = (self.wind_dir - self.heading) % 360
        return [nmea.build("KLMWV", "%.1f" % awa, "R", "%.1f" % self.wind_speed, "N", "A")]

    def rmb(self):
        name, wlat, wlon = WAYPOINTS[self.wp]
        rng, brg = self.range_bearing(wlat, wlon)
        lat, ns = nmea.from_degrees(wlat, True)
        lon, ew = nmea.from_degrees(wlon, False)
        err = (brg - self.heading + 540) % 360 - 180
        return [nmea.build("KLRMB", "A", "0.00", "L" if err < 0 else "R", self.prev_wp[0], name,
                           lat, ns, lon, ew, "%.2f" % rng, "%.1f" % brg, "%.1f" % self.sog,
                           "A" if rng < 0.05 else "V", "A")]

    def xdr_imu(self):
        return [nmea.build("KLXDR", "A", "%.1f" % self.pitch, "D", "PITCH", "A", "%.1f" % self.roll, "D", "ROLL")]

    def xdr_env(self):
        return [nmea.build("KLXDR", "C", "%.1f" % self.board, "C", "BOARD", "U", "%.2f" % self.batt, "V", "BATT",
                           "U", "%.2f" % self.solar, "V", "SOLAR")]

    def xdr_ctrl(self):
        return [nmea.build("KLXDR", "A", "%.1f" % self.wing, "D", "WING")]

    def source(self, name):
        table = {
            "RMC": self.rmc, "GGA": self.gga, "HDT": self.hdt, "ROT": self.rot_,
            "RSA": self.rsa, "MWV": self.mwv, "RMB": self.rmb,
            "XDR.IMU": self.xdr_imu, "XDR.ENV": self.xdr_env, "XDR.CTRL": self.xdr_ctrl,
        }
        return table.get(name)

    def generate(self, name):
        with self._lock:
            return self.source(name)()


#
# Payload data: tag dictionary, "microSD" log, Iridium uplink bucket
class DataStore:
    def __init__(self, sd_path, uplink_every, bucket_bytes, refill_per_hour):
        self.tags = {}                  # name -> index (lives until the CM reboots)
        self.sd = open(sd_path, "a", buffering=1)
        self.uplink_every = uplink_every
        self.capacity = float(bucket_bytes)
        self.tokens = float(bucket_bytes)
        self.refill = refill_per_hour / 3600.0
        self.pending = {}               # tag -> (value, cm timestamp)
        self.last_refill = time.time()
        self.next_uplink = time.time() + uplink_every
        self.lock = threading.Lock()
        self.sd.write("# cmsim boot %s\n" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))

    def _index(self, tag):
        if tag not in self.tags:
            if len(self.tags) >= MAX_TAGS:
                return None
            self.tags[tag] = len(self.tags)
            self.sd.write("D %.3f %d %s\n" % (time.time(), self.tags[tag], tag))   # tag definition
        return self.tags[tag]

    def _refill(self):
        now = time.time()
        self.tokens = min(self.capacity, self.tokens + (now - self.last_refill) * self.refill)
        self.last_refill = now

    def pending_cost(self):
        return len(self.pending) * TAG_UPLINK_COST

    def save(self, pairs):
        """Returns None on success, else a NAK reason. All or nothing."""
        with self.lock:
            new = [t for t, _ in pairs if t not in self.tags]
            if len(self.tags) + len(set(new)) > MAX_TAGS:
                return "TAGS"
            now = time.time()
            for tag, val in pairs:
                self.sd.write("V %.3f %d %r\n" % (now, self._index(tag), val))
            return None

    def xmit(self, pairs):
        with self.lock:
            self._refill()
            extra = len({t for t, _ in pairs if t not in self.pending}) * TAG_UPLINK_COST
            if self.pending_cost() + extra > self.tokens:
                return "QUOTA"
        err = self.save(pairs)          # XMIT always lands on microSD too
        if err:
            return err
        with self.lock:
            now = time.time()
            for tag, val in pairs:
                self.pending[tag] = (val, now)       # latest value wins
            return None

    def note(self, text):
        with self.lock:
            self.sd.write("N %.3f %s\n" % (time.time(), text))

    def quota(self):
        with self.lock:
            self._refill()
            return int(self.tokens), self.pending_cost(), max(0, int(self.next_uplink - time.time()))

    def uplink_loop(self):
        while True:
            time.sleep(1)
            with self.lock:
                if time.time() < self.next_uplink:
                    continue
                self.next_uplink = time.time() + self.uplink_every
                if not self.pending:
                    continue
                self._refill()
                msg = b""
                for tag, (val, _) in sorted(self.pending.items()):
                    idx = self.tags[tag]
                    msg += struct.pack("<BB8s", TLV_PAYLOAD_TAGDEF, idx, tag.encode())
                    msg += struct.pack("<BBf", TLV_PAYLOAD_VALUE, idx, val)
                self.tokens -= len(msg)
                summary = ", ".join("%s=%g" % (t, v) for t, (v, _) in sorted(self.pending.items()))
                self.pending.clear()
                left = int(self.tokens)
            log("IRIDIUM UPLINK %d bytes [%s]  (%d bytes left)" % (len(msg), summary, left))
            log("  SBD payload: %s" % msg.hex())


#
# One connected payload
class Session:
    def __init__(self, sock, boat, store, baud, verbose):
        self.sock = sock
        self.boat = boat
        self.store = store
        self.char_budget = 0.8 * baud / 10          # chars/s we let subscriptions use
        self.verbose = verbose
        self.subs = {}                              # name -> [rate, next_due, est_chars]
        self.lock = threading.Lock()                # guards self.subs
        self.wlock = threading.Lock()               # guards the socket writes
        self.alive = True
        self.last_hb = time.time()
        self.hb_warned = False

    def send(self, line):
        try:
            with self.wlock:
                self.sock.sendall(line.encode("ascii"))
        except OSError:
            self.alive = False

    def reply(self, *fields):
        line = nmea.build(TALKER, *fields)
        if self.verbose:
            log("-> " + line.strip())
        self.send(line)

    def run(self):
        threading.Thread(target=self.sender, daemon=True).start()
        self.reply("HELLO", "CMSIM", 1)
        buf = b""
        self.sock.settimeout(0.5)
        while self.alive:
            try:
                chunk = self.sock.recv(512)
            except socket.timeout:
                self.watchdog()
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle(raw.decode("ascii", "replace").strip())
            if len(buf) > 256:
                buf = b""
                self.reply("NAK", "?", "LEN")
        self.alive = False

    def watchdog(self):
        if time.time() - self.last_hb > HB_TIMEOUT and not self.hb_warned:
            self.hb_warned = True
            log("payload silent for %ds -- real CM would power-cycle it" % HB_TIMEOUT)

    def sender(self):
        while self.alive:
            now = time.time()
            due = []
            with self.lock:
                for name, s in self.subs.items():
                    if now >= s[1]:
                        due.append(name)
                        s[1] += 1.0 / s[0]
                        if s[1] < now:                  # fell behind: don't burst
                            s[1] = now + 1.0 / s[0]
            for name in due:
                for line in self.boat.generate(name):
                    self.send(line)
            time.sleep(0.01)

    def handle(self, text):
        if not text:
            return
        if self.verbose:
            log("<- " + text)
        try:
            f = nmea.parse(text)
        except nmea.NMEAError as e:
            reason = {"bad checksum": "CSUM", "too long": "LEN"}.get(str(e), "SYNTAX")
            return self.reply("NAK", "?", reason)
        if f[0] != TALKER or len(f) < 2:
            return self.reply("NAK", "?", "TALKER")
        self.last_hb = time.time()
        self.hb_warned = False
        verb, args = f[1], f[2:]
        handler = getattr(self, "cmd_" + verb.lower(), None)
        if handler is None:
            return self.reply("NAK", verb, "UNKNOWN")
        handler(verb, args)

    def cmd_hb(self, verb, args):
        pass                                          # silence is golden

    def cmd_sub(self, verb, args):
        if len(args) != 2:
            return self.reply("NAK", verb, "SYNTAX")
        name = args[0].upper()
        if self.boat.source(name) is None:
            return self.reply("NAK", verb, name, "UNKNOWN")
        try:
            rate = float(args[1])
        except ValueError:
            return self.reply("NAK", verb, name, "SYNTAX")
        if not 0 < rate <= MAX_RATE:
            return self.reply("NAK", verb, name, "RATE")
        est = sum(len(l) for l in self.boat.generate(name))
        with self.lock:
            used = sum(r * c for n, (r, _, c) in self.subs.items() if n != name)
            if used + rate * est > self.char_budget:
                return self.reply("NAK", verb, name, "BUDGET")
            self.subs[name] = [rate, time.time(), est]
        log("payload subscribed to %s at %g Hz (link %.0f%% used)" %
            (name, rate, 100 * (used + rate * est) / self.char_budget))
        self.reply("ACK", verb, name)

    def cmd_unsub(self, verb, args):
        name = args[0].upper() if args else ""
        with self.lock:
            found = self.subs.pop(name, None)
        self.reply("ACK" if found else "NAK", verb, name, *([] if found else ["UNKNOWN"]))

    def cmd_get(self, verb, args):
        name = args[0].upper() if args else ""
        if self.boat.source(name) is None:
            return self.reply("NAK", verb, name, "UNKNOWN")
        lines = self.boat.generate(name)
        for line in lines:
            self.send(line)
        names = []
        if name.startswith("XDR."):                   # tell the payload which transducers that was
            for line in lines:
                q = nmea.parse(line)[1:]
                names += q[3::4]
        self.reply("ACK", verb, name, *names)

    def _pairs(self, verb, args):
        if not args or len(args) % 2:
            self.reply("NAK", verb, "SYNTAX")
            return None
        pairs = []
        for tag, val in zip(args[0::2], args[1::2]):
            if not TAG_RE.match(tag):
                self.reply("NAK", verb, tag, "TAG")
                return None
            try:
                v = float(val)
                if not math.isfinite(v):
                    raise ValueError
            except ValueError:
                self.reply("NAK", verb, tag, "VALUE")
                return None
            pairs.append((tag, v))
        return pairs

    def cmd_save(self, verb, args):
        pairs = self._pairs(verb, args)
        if pairs is None:
            return
        err = self.store.save(pairs)
        if err:
            return self.reply("NAK", verb, err)
        if self.verbose:
            log("saved " + ", ".join("%s=%g" % p for p in pairs))
        self.reply("ACK", verb)

    def cmd_xmit(self, verb, args):
        pairs = self._pairs(verb, args)
        if pairs is None:
            return
        err = self.store.xmit(pairs)
        if err == "QUOTA":
            self.store.save(pairs)                    # never lose data, just don't send it
        if err:
            return self.reply("NAK", verb, err)
        self.reply("ACK", verb)

    def cmd_note(self, verb, args):
        self.store.note(",".join(args))
        self.reply("ACK", verb)

    def cmd_quota(self, verb, args):
        self.reply("QUOTA", *self.store.quota())


#
def main():
    ap = argparse.ArgumentParser(description="Kalopa command module simulator (payload side)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5010)
    ap.add_argument("--baud", type=int, default=38400, help="link speed used for the subscription budget")
    ap.add_argument("--uplink", type=int, default=60, help="seconds between simulated Iridium uplinks")
    ap.add_argument("--bucket", type=int, default=340, help="Iridium byte bucket size")
    ap.add_argument("--refill", type=int, default=340, help="bucket refill, bytes per hour")
    ap.add_argument("--sdlog", default="cmsim_sd.log", help="file standing in for the microSD card")
    ap.add_argument("--verbose", "-v", action="store_true", help="show every command and reply")
    a = ap.parse_args()

    boat = Boat()
    store = DataStore(a.sdlog, a.uplink, a.bucket, a.refill)

    def physics():
        last = time.time()
        while True:
            time.sleep(0.05)
            now = time.time()
            boat.step(now - last)
            last = now

    threading.Thread(target=physics, daemon=True).start()
    threading.Thread(target=store.uplink_loop, daemon=True).start()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.host, a.port))
    srv.listen(1)
    log("listening on %s:%d (one payload at a time), microSD log -> %s" % (a.host, a.port, a.sdlog))
    try:
        while True:
            sock, addr = srv.accept()
            log("payload connected from %s:%d" % addr)
            Session(sock, boat, store, a.baud, a.verbose).run()
            sock.close()
            log("payload disconnected")
    except KeyboardInterrupt:
        log("bye")


if __name__ == "__main__":
    main()
