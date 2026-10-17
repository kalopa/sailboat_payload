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
# NMEA 0183 framing helpers shared by the payload library and the simulator.
#
# A sentence is  $<fields joined by commas>*<two hex digit XOR # checksum>\r\n
# and may not exceed 82 characters including the '$' and the CRLF.
#
MAX_LEN = 82


class NMEAError(ValueError):
    """Raised for malformed sentences (no '$', no checksum, bad checksum, too long)."""


def checksum(body):
    """XOR of every character between '$' and '*', as two upper-case hex digits."""
    x = 0
    for c in body.encode("ascii"):
        x ^= c
    return "%02X" % x


def build(*fields):
    """Build a complete sentence (with CRLF) from its fields.

    build("PKLP", "SAVE", "TEMP", 20.0) -> "$PKLP,SAVE,TEMP,20.0*hh\\r\\n"
    """
    body = ",".join(str(f) for f in fields)
    for bad in "$*\r\n":
        if bad in body:
            raise NMEAError("field contains reserved character %r" % bad)
    line = "$%s*%s\r\n" % (body, checksum(body))
    if len(line) > MAX_LEN:
        raise NMEAError("sentence is %d chars, limit is %d" % (len(line), MAX_LEN))
    return line


def parse(line):
    """Validate a sentence and return its fields as a list of strings.

    "$GPRMC,...*hh" -> ["GPRMC", ...]. Raises NMEAError on any problem.
    """
    line = line.strip()
    if len(line) + 2 > MAX_LEN:
        raise NMEAError("too long")
    if not line.startswith("$"):
        raise NMEAError("no leading $")
    if "*" not in line:
        raise NMEAError("no checksum")
    body, cs = line[1:].rsplit("*", 1)
    if cs.upper() != checksum(body):
        raise NMEAError("bad checksum")
    return body.split(",")


def sentence_id(fields):
    """The sentence type without the talker: GPRMC -> RMC, KLXDR -> XDR.

    Proprietary sentences ($P...) are returned whole, e.g. PKLP.
    """
    head = fields[0]
    if head.startswith("P"):
        return head
    return head[2:]


def to_degrees(value, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm plus N/S/E/W -> signed decimal degrees."""
    if not value:
        return None
    v = float(value)
    deg = int(v // 100)
    result = deg + (v - deg * 100) / 60.0
    return -result if hemi in ("S", "W") else result


def from_degrees(value, is_lat):
    """Signed decimal degrees -> (ddmm.mmmm string, hemisphere letter)."""
    hemi = ("N" if value >= 0 else "S") if is_lat else ("E" if value >= 0 else "W")
    value = abs(value)
    deg = int(value)
    minutes = (value - deg) * 60.0
    if is_lat:
        return "%02d%07.4f" % (deg, minutes), hemi
    return "%03d%07.4f" % (deg, minutes), hemi
