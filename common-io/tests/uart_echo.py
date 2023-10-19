#!/usr/bin/env python3

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
