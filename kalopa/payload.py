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
# The real thing: talk to the command module over the payload UART.
#
#    from kalopa import payload
#    cm = payload.connect()
#
# If the environment variable KALOPA_PAYLOAD is set to tcp://host:port,
# connect() goes to the simulator instead, so the same program runs
# unchanged on a laptop and on the boat:
#
#    KALOPA_PAYLOAD=tcp://localhost:5010 python3 my_payload.py
#
import os

from ._core import Payload, PayloadError, Reading  # noqa: F401
from .transport import SerialTransport, TCPTransport

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 38400


def connect(device=None, baud=None, **kw):
    url = os.environ.get("KALOPA_PAYLOAD", "")
    if device is None and url.startswith("tcp://"):
        host, _, port = url[6:].partition(":")
        return Payload(TCPTransport(host or "localhost", int(port or 5010)), **kw)
    return Payload(SerialTransport(device or DEFAULT_DEVICE, baud or DEFAULT_BAUD), **kw)
