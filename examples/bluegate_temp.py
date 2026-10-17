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
# BlueGate example: read the Atlas EZO-RTD and report it to the command module.
#
# On the boat:          python3 bluegate_temp.py
# Against simulator:    KALOPA_PAYLOAD=tcp://localhost:5010 python3 bluegate_temp.py
# No probe attached:    add --fake-sensor
#
import random
import sys
import time

from kalopa import payload, PayloadError

TEAM_NAME = "Alpha"
SENSOR_ADDRESS = 0x66
SAVE_EVERY = 5        # seconds
XMIT_EVERY = 60       # seconds -- satellite bytes cost money!


def read_atlas_sensor(address):
    """Return the temperature in C, or None if the read failed."""
    if "--fake-sensor" in sys.argv:
        return round(12.0 + random.uniform(-0.3, 0.3), 3)
    from smbus2 import SMBus, i2c_msg
    with SMBus(1) as bus:
        bus.i2c_rdwr(i2c_msg.write(address, b"R\x00"))
        time.sleep(1.0)                       # the RTD needs 600 ms
        rd = i2c_msg.read(address, 31)
        bus.i2c_rdwr(rd)
        data = bytes(rd)
    if data[0] != 1:
        return None
    return float(data[1:].split(b"\x00")[0].decode("ascii"))


with payload.connect() as cm:
    print("Connected to the command module")
    cm.subscribe("RMC", 1)            # where are we?
    cm.subscribe("XDR.ENV", 0.2)      # boat battery etc. every 5 s
    cm.note("team %s payload started" % TEAM_NAME)

    last_xmit = 0
    while cm.connected:
        temp = read_atlas_sensor(SENSOR_ADDRESS)
        if temp is None:
            print("sensor read failed")
        else:
            cm.save(TEMP=temp)

            if time.time() - last_xmit >= XMIT_EVERY:
                try:
                    cm.xmit(TEMP=temp)
                    last_xmit = time.time()
                except PayloadError as e:
                    if e.reason == "QUOTA":
                        print("satellite allowance used up -- saved locally only")
                    else:
                        raise

        pos = cm.position
        where = "%.4f, %.4f" % pos if pos else "no fix yet"
        print("TEMP %s C at %s, boat battery %s V" % (temp, where, cm.value("BATT")))
        time.sleep(SAVE_EVERY)
