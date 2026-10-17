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
# SPDX-FileCopyrightText: 2026 Kalopa Robotics Limited
# SPDX-License-Identifier: Apache-2.0
"""End-to-end tests: start the command module simulator, talk to it with the library."""

import socket
import subprocess
import sys
import time

import pytest

from kalopa import fake_payload, nmea, PayloadError


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def cm(tmp_path):
    port = free_port()
    sim = subprocess.Popen(
        [sys.executable, "-m", "kalopa.cmsim", "--host", "127.0.0.1", "--port", str(port),
         "--sdlog", str(tmp_path / "sd.log"), "--uplink", "2", "--bucket", "40"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.time() + 10
    while True:
        try:
            link = fake_payload.connect("127.0.0.1", port)
            break
        except OSError:
            if time.time() > deadline:
                sim.kill()
                raise
            time.sleep(0.1)
    link.sdlog = tmp_path / "sd.log"
    yield link
    link.close()
    sim.terminate()
    sim.wait(5)


def test_nmea_round_trip():
    line = nmea.build("PKLP", "SAVE", "TEMP", "20.0")
    assert nmea.parse(line) == ["PKLP", "SAVE", "TEMP", "20.0"]
    with pytest.raises(nmea.NMEAError):
        nmea.parse("$PKLP,SAVE,TEMP,20.0*00")


def test_subscribe_and_position(cm):
    cm.subscribe("RMC", 5)
    assert cm.wait_for("RMC", timeout=3) is not None
    lat, lon = cm.position
    assert 53 < lat < 54 and -10 < lon < -8


def test_get_xdr_group(cm):
    values = cm.get("XDR.ENV")
    assert set(values) == {"BOARD", "BATT", "SOLAR"}


def test_naks(cm):
    with pytest.raises(PayloadError) as e:
        cm.subscribe("RMC", 50)
    assert e.value.reason == "RATE"
    with pytest.raises(PayloadError) as e:
        cm.subscribe("NOPE", 1)
    assert e.value.reason == "UNKNOWN"
    with pytest.raises(PayloadError) as e:
        cm.save(TOOLONGTAGNAME=1)
    assert e.value.reason == "TAG"


def test_save_writes_log(cm):
    cm.save(TEMP=20.5, PH=7.1)
    cm.note("hello")
    time.sleep(0.2)
    log = cm.sdlog.read_text()
    assert " TEMP" in log and "20.5" in log and "hello" in log


def test_xmit_quota_and_uplink(cm):
    cm.xmit(TEMP=1.0)
    cm.xmit(TEMP=2.0)                       # coalesced: still one tag pending
    assert cm.quota()["pending"] == 16
    cm.xmit(PH=7.0)                          # 32 of 40 bytes
    with pytest.raises(PayloadError) as e:
        cm.xmit(COND=500)
    assert e.value.reason == "QUOTA"
    time.sleep(3)                            # uplink every 2 s in the fixture
    assert cm.quota()["pending"] == 0
