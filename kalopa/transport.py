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
# Byte transports between a payload and the command module.
#
# Both classes offer the same three calls, so nothing above this layer
# knows (or cares) whether it is talking to a real UART or the simulator:
#
#    recv()        -> bytes, or None if nothing arrived within ~0.5 s;
#                     raises EOFError when the link is gone
#    write(data)   -> send bytes
#    close()
#
import socket

POLL = 0.5


class TCPTransport:
    """TCP socket to the command module simulator (kalopa.cmsim)."""

    def __init__(self, host, port):
        self.name = "tcp://%s:%d" % (host, port)
        self._sock = socket.create_connection((host, port), timeout=5)
        self._sock.settimeout(POLL)

    def recv(self):
        try:
            data = self._sock.recv(512)
        except socket.timeout:
            return None
        except OSError as e:
            raise EOFError(str(e))
        if not data:
            raise EOFError("simulator closed the connection")
        return data

    def write(self, data):
        self._sock.sendall(data)

    def close(self):
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()


class SerialTransport:
    """The real payload UART on the vessel (normally /dev/ttyAMA0)."""

    def __init__(self, device, baud):
        try:
            import serial  # pyserial
        except ImportError:
            raise ImportError("pyserial is needed for the real port: pip install pyserial")
        self.name = "%s@%d" % (device, baud)
        self._ser = serial.Serial(device, baud, timeout=POLL)

    def recv(self):
        try:
            data = self._ser.read(self._ser.in_waiting or 1)
        except Exception as e:  # pyserial raises SerialException on unplug
            raise EOFError(str(e))
        return data or None

    def write(self, data):
        self._ser.write(data)

    def close(self):
        self._ser.close()
