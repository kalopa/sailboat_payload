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
# The payload side of the Kalopa command-module link.
#
# Concurrency model (the short version for customers):
#
#   * A background thread reads everything the command module sends and
#     keeps the LATEST copy of each sentence. Telemetry is a stream of
#     samples, not a queue of events: if you miss one, the next is along
#     in a fraction of a second, and you only ever want the newest.
#   * Commands (subscribe, save, xmit, ...) are ordinary blocking calls.
#     They send one sentence, wait for the matching ACK or NAK, and either
#     return or raise PayloadError. No callbacks, no asyncio.
#   * A heartbeat is sent automatically so the command module knows the
#     payload is alive.
#
# So a payload program is just a normal loop:
#
#    with payload.connect() as cm:
#        cm.subscribe("RMC", 1)
#        while True:
#            t = read_my_sensor()
#            cm.save(TEMP=t)
#            print(cm.position)
#            time.sleep(5)
#
# For people who prefer events, on() registers a callback and wait_for()
# blocks until the next fresh copy of a sentence arrives.
#
import queue
import threading
import time

from . import nmea

TALKER = "PKLP"
HEARTBEAT_SECONDS = 10


class PayloadError(Exception):
    """A command was refused (NAK) or got no reply."""

    def __init__(self, message, reason=None):
        super().__init__(message)
        self.reason = reason


class Reading:
    """One received sentence: its type, its fields, and when it arrived."""

    __slots__ = ("name", "fields", "time")

    def __init__(self, name, fields, when):
        self.name = name
        self.fields = fields
        self.time = when

    @property
    def age(self):
        return time.time() - self.time

    def __repr__(self):
        return "Reading(%s, %s, age=%.1fs)" % (self.name, self.fields, self.age)


class Payload:
    def __init__(self, transport, reply_timeout=2.0, heartbeat=True):
        self._t = transport
        self.reply_timeout = reply_timeout
        self._write_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._replies = queue.Queue()
        self._cond = threading.Condition()
        self._latest = {}       # "RMC" -> Reading
        self._xdr = {}          # "BATT" -> Reading (one per transducer name)
        self._callbacks = {}
        self._running = True
        self.closed_reason = None
        self.bad_sentences = 0
        self.hello = None

        self._reader = threading.Thread(target=self._read_loop, name="kalopa-rx", daemon=True)
        self._reader.start()
        if heartbeat:
            threading.Thread(target=self._heartbeat_loop, name="kalopa-hb", daemon=True).start()

    #
    # Commands: blocking, one at a time, ACK returns / NAK raises
    def subscribe(self, name, hz):
        """Ask for a sentence (e.g. "RMC", "HDT", "XDR.ENV") at hz, up to 10 Hz."""
        self._command("SUB", name.upper(), _num(hz))

    def unsubscribe(self, name):
        self._command("UNSUB", name.upper())

    def get(self, name, timeout=None):
        """One-shot request; returns the fresh Reading, or for XDR groups a {name: value} dict."""
        name = name.upper()
        rep = self._command("GET", name, timeout=timeout)
        # The data sentence is always sent before the ACK, so it is already here.
        if name.startswith("XDR."):
            # ACK,GET,XDR.IMU,PITCH,ROLL -> {"PITCH": -1.3, "ROLL": 7.9}
            return {n: self.value(n) for n in rep[3:]}
        return self._latest.get(name)

    def save(self, values=None, **kw):
        """Log tagged numbers to the command module's microSD card.

        cm.save(TEMP=20.1, PH=7.02)  or  cm.save({"TEMP": 20.1})
        Tags: up to 8 chars, A-Z 0-9 _, starting with a letter.
        """
        self._command("SAVE", *_pairs(values, kw))

    def xmit(self, values=None, **kw):
        """Queue tagged numbers for the next Iridium uplink (also saved to microSD).

        Only the most recent value of each tag is sent. Raises PayloadError
        with reason "QUOTA" if your satellite byte allowance is used up.
        """
        self._command("XMIT", *_pairs(values, kw))

    def note(self, text):
        """Save a short free-text line to microSD (never transmitted)."""
        clean = "".join(c for c in str(text) if c not in "$*,\r\n")
        self._command("NOTE", clean)

    def quota(self):
        """Satellite allowance: {'available': bytes, 'pending': bytes, 'next_uplink': secs}."""
        rep = self._command("QUOTA", expect="QUOTA")
        return {"available": int(rep[1]), "pending": int(rep[2]), "next_uplink": int(rep[3])}

    #
    # Telemetry: always the latest value, never blocks
    def latest(self, name):
        """Latest Reading of a sentence ("RMC", "HDT", ...) or None."""
        return self._latest.get(name.upper())

    def value(self, name, max_age=None):
        """Latest number for an XDR transducer ("BATT", "PITCH", "WING", ...) or None."""
        r = self._xdr.get(name.upper())
        if r is None or (max_age is not None and r.age > max_age):
            return None
        return float(r.fields[1])

    @property
    def position(self):
        """(lat, lon) in decimal degrees from the latest RMC, or None."""
        r = self._latest.get("RMC")
        if r is None or r.fields[1] != "A":
            return None
        f = r.fields
        return nmea.to_degrees(f[2], f[3]), nmea.to_degrees(f[4], f[5])

    @property
    def heading(self):
        """True heading in degrees from the latest HDT, or None."""
        r = self._latest.get("HDT")
        return float(r.fields[0]) if r and r.fields[0] else None

    def wait_for(self, name, timeout=None):
        """Block until a NEW copy of a sentence arrives; return it (None on timeout)."""
        name = name.upper()
        start = time.time()
        with self._cond:
            ok = self._cond.wait_for(
                lambda: (name in self._latest and self._latest[name].time >= start)
                or not self._running,
                timeout,
            )
        return self._latest.get(name) if ok and self._running else None

    def on(self, name, callback):
        """Call callback(reading) for every arriving sentence of this type.

        Callbacks run on the receive thread: keep them short and never
        call commands (save, xmit, ...) from inside one.
        """
        self._callbacks.setdefault(name.upper(), []).append(callback)

    def close(self):
        self._running = False
        self._t.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def connected(self):
        return self._running

    #
    # Internals
    def _send(self, *fields):
        line = nmea.build(TALKER, *fields)
        with self._write_lock:
            self._t.write(line.encode("ascii"))

    def _command(self, verb, *args, expect=None, timeout=None):
        if not self._running:
            raise PayloadError("not connected: %s" % self.closed_reason, reason="LINK")
        timeout = self.reply_timeout if timeout is None else timeout
        with self._cmd_lock:
            while not self._replies.empty():          # discard anything stale
                self._replies.get_nowait()
            self._send(verb, *args)
            deadline = time.time() + timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    raise PayloadError("no reply to %s" % verb, reason="TIMEOUT")
                try:
                    rep = self._replies.get(timeout=remaining)
                except queue.Empty:
                    continue
                if expect and rep[0] == expect:
                    return rep
                if rep[0] in ("ACK", "NAK") and len(rep) > 1 and rep[1] in (verb, "?"):
                    if rep[0] == "NAK":
                        reason = rep[-1]
                        raise PayloadError("%s %s refused: %s" % (verb, ",".join(map(str, args)), reason),
                                           reason=reason)
                    return rep

    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._t.recv()
            except EOFError as e:
                self.closed_reason = str(e)
                break
            except Exception as e:
                if self._running:
                    self.closed_reason = str(e)
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle(raw.decode("ascii", "replace"))
            if len(buf) > 1024:                       # line noise, no newline
                buf = b""
        self._running = False
        with self._cond:
            self._cond.notify_all()

    def _handle(self, text):
        text = text.strip()
        if not text:
            return
        try:
            fields = nmea.parse(text)
        except nmea.NMEAError:
            self.bad_sentences += 1
            return
        sid = nmea.sentence_id(fields)
        now = time.time()

        if sid == TALKER:
            if len(fields) > 1 and fields[1] == "HELLO":
                self.hello = fields[2:]
            elif len(fields) > 1:
                self._replies.put(fields[1:])
            return

        reading = Reading(sid, fields[1:], now)
        if sid == "XDR":
            q = fields[1:]
            for i in range(0, len(q) - 3, 4):
                xtype, val, unit, xname = q[i:i + 4]
                self._xdr[xname] = Reading(xname, [xtype, val, unit], now)
        with self._cond:
            self._latest[sid] = reading
            self._cond.notify_all()

        for cb in self._callbacks.get(sid, []):
            try:
                cb(reading)
            except Exception as e:                    # never kill the reader
                print("kalopa: callback for %s raised %r" % (sid, e))

    def _heartbeat_loop(self):
        while self._running:
            time.sleep(HEARTBEAT_SECONDS)
            if self._running:
                try:
                    self._send("HB")
                except Exception:
                    pass


def _num(v):
    return format(float(v), ".6g")


def _pairs(values, kw):
    items = dict(values or {})
    items.update(kw)
    if not items:
        raise ValueError("nothing to send")
    out = []
    for tag, val in items.items():
        out += [str(tag).upper(), _num(val)]
    return out
