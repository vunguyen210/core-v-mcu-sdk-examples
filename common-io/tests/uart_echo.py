#!/usr/bin/env python3

# Copyright 2023 AWS
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
# SPDX-License-Identifier: Apache-2.0

import argparse
import serial
import re

def uartThread( port: str ) -> None:
    uart = serial.Serial()
    uart.baudrate = 115200
    uart.stopbits = 1
    uart.xonxoff = False
    uart.bytesize = 8
    uart.timeout = 0.01
    uart.port = args.port
    uart.open()
    print("UART ready")
    while True:
        c = uart.readline()
        if b'\n' not in c:
            continue
        try:
            if( b"Baudrate: " in c ):
                uart.close()
                uart.baudrate = re.match(r"Baudrate: (\d+)\n", c.decode(errors='replace')).groups()[1]
                print(f"Baudrate: {uart.baudrate}")
                uart.open()
            else:
                print(c.decode(errors='replace'), end ='', flush=True)
                uart.write(c)
        except:
            pass

class _Args:
    port: str

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This script opens a UART session with the CORE-V running the Common-IO UART tests.')
    parser.add_argument('--port','-p',type=str,help='Location of the UART port (e.g. COM3 or /dev/ttyUSB1)',required=True)
    args : _Args = parser.parse_args()
    uartThread(port=args.port)
