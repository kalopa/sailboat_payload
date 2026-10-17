# Kalopa Payload Library and Command Module simulator

The Kalopa fleet of autonomous sailboats (ASVs) provide an RS422 serial
interface and twelve volt (12v) power to a payload in the moon pool.
The connector is a six pin Weipu SP21.

The onboard Command Module (CM) can provide GPS and other telemetry
information to a client payload.
In addition, the client payload can use the CM communication layer to send
short messages ashore, and receive short messages from Mission Control.

This library allows a prospective client to test their payload interface
by simulating the CM.

```
kalopa/
  payload.py       real UART (/dev/ttyAMA0, 38400) -- or the simulator if KALOPA_PAYLOAD=tcp://host:port
  fake_payload.py  always the simulator (localhost:5010)
  _core.py         the shared Payload class: reader thread, commands, latest-value store
  transport.py     SerialTransport / TCPTransport (recv/write/close)
  nmea.py          checksum, build, parse
  cmsim.py         command module simulator
examples/
  bluegate_temp.py  plain-loop style (recommended)
  event_style.py    callbacks + wait_for
```

## Running

```
python3 -m kalopa.cmsim -v                    # terminal 1: the "boat"
KALOPA_PAYLOAD=tcp://localhost:5010 \
  python3 examples/bluegate_temp.py --fake-sensor   # terminal 2: the payload
```

Simulator options: `--port`, `--baud` (sets the subscription budget), `--uplink` (seconds
between Iridium uplinks), `--bucket` / `--refill` (Iridium bytes, refill per hour),
`--sdlog` (file standing in for the microSD card), `-v` (show every command and reply).

The real port needs `pip install pyserial`; the simulator path has no dependencies.

## Writing a payload

```python
from kalopa import payload, PayloadError

with payload.connect() as cm:
    cm.subscribe("RMC", 1)
    cm.subscribe("XDR.ENV", 0.2)
    while cm.connected:
        cm.save(TEMP=read_sensor())
        print(cm.position, cm.heading, cm.value("BATT"))
        time.sleep(5)
```

* **Telemetry** arrives in the background. `latest()`, `value()`, `position`, `heading`
  return the newest copy immediately and never block.
* **Commands** (`subscribe`, `unsubscribe`, `get`, `save`, `xmit`, `note`, `quota`)
  block until the command module ACKs, or raise `PayloadError` with `.reason` set.
* `wait_for(name)` blocks for the next fresh sentence; `on(name, fn)` registers a callback
  (runs on the receive thread: read data only, no commands).
* A heartbeat goes out every 10 s automatically.

## Protocol as implemented

Payload to CM (talker `PKLP`, max 82 chars, checksum required):

| Sentence | Reply |
|---|---|
| `$PKLP,SUB,<name>,<hz>` | `ACK,SUB,<name>` / `NAK,SUB,<name>,RATE\|BUDGET\|UNKNOWN` |
| `$PKLP,UNSUB,<name>` | `ACK,UNSUB,<name>` |
| `$PKLP,GET,<name>` | the sentence, then `ACK,GET,<name>[,<xdr names>]` |
| `$PKLP,SAVE,<tag>,<val>[,<tag>,<val>...]` | `ACK,SAVE` / `NAK,SAVE,TAGS` / `NAK,SAVE,<tag>,TAG\|VALUE` |
| `$PKLP,XMIT,<tag>,<val>[,...]` | `ACK,XMIT` / `NAK,XMIT,QUOTA` (still saved to microSD) |
| `$PKLP,NOTE,<text>` | `ACK,NOTE` (microSD only) |
| `$PKLP,QUOTA` | `$PKLP,QUOTA,<bytes available>,<bytes pending>,<secs to uplink>` |
| `$PKLP,HB` | none (30 s silence -> CM would power-cycle the payload) |
| anything malformed | `NAK,?,CSUM\|LEN\|SYNTAX\|TALKER` |

CM to payload: `$PKLP,HELLO,CMSIM,1` on connect, then subscribed sentences:
`GPRMC GPGGA KLHDT KLROT KLRSA KLMWV KLRMB` and `KLXDR` groups
`XDR.IMU` (PITCH ROLL), `XDR.ENV` (BOARD BATT SOLAR), `XDR.CTRL` (WING).

Rules: max 10 Hz per stream; total subscribed traffic limited to 80% of the link;
tags `[A-Z][A-Z0-9_]{0,7}`, 32 per boot, all-or-nothing per sentence.

## Storage and Iridium

microSD log (`--sdlog`): `D <time> <index> <tag>` the first time a tag is seen, then
`V <time> <index> <value>` and `N <time> <text>`, so the log decodes on its own.

XMIT values are coalesced per tag (latest wins) and sent at the next uplink as TLVs:
`0x60 idx name[8]` + `0x61 idx float32le` = 16 bytes per tag, charged against the byte
bucket. **0x60/0x61 are placeholders** until they are allocated in the mission-enums gem.

## Copyright Notice

Copyright 2026, Kalopa Robotics Limited

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
