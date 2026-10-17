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
# Same API as kalopa.payload, but always talks to the simulator over TCP.
#
#    from kalopa import fake_payload as payload
#    cm = payload.connect()                      # localhost:5010
#
# Start the simulator first:  python3 -m kalopa.cmsim
#
from ._core import Payload, PayloadError, Reading  # noqa: F401
from .transport import TCPTransport

DEFAULT_HOST = "localhost"
DEFAULT_PORT = 5010


def connect(host=DEFAULT_HOST, port=DEFAULT_PORT, **kw):
    return Payload(TCPTransport(host, port), **kw)
