#!/usr/bin/env python

import serial
import sys

ser = serial.Serial(
    port = sys.argv[1],
    baudrate = 9600,
    bytesize=serial.EIGHTBITS,
    stopbits = serial.STOPBITS_ONE,
    parity = serial.PARITY_NONE,
    timeout = 10
)

while True:
    sys.stdout.write(ser.read(1))
    sys.stdout.flush()
