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
# Event-driven style, for people who prefer it.
#
# Callbacks run on the library's receive thread, so they should only
# look at data. Anything that sends a command (save, xmit, ...) belongs
# in the main loop.
#
#    python3 event_style.py           (talks to the simulator on localhost:5010)
#
from kalopa import fake_payload as payload

with payload.connect() as cm:
    cm.on("HDT", lambda r: print("heading now %s" % r.fields[0]))
    cm.subscribe("HDT", 2)
    cm.subscribe("RMB", 1)

    # wait_for() blocks until the NEXT copy of a sentence arrives
    for _ in range(5):
        rmb = cm.wait_for("RMB", timeout=5)
        if rmb:
            dest, rng, brg = rmb.fields[4], rmb.fields[9], rmb.fields[10]
            print("next waypoint %s: %s nm, bearing %s" % (dest, rng, brg))
            cm.save(RANGE=float(rng))
