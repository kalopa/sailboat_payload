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
# Kalopa Robotics payload interface.
#
#   from kalopa import payload        # real UART (or KALOPA_PAYLOAD=tcp://...)
#   from kalopa import fake_payload   # always the TCP simulator
#
from ._core import PayloadError, Reading  # noqa: F401

__version__ = "0.1.0"
