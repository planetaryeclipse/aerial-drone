#!/bin/python3

import serial
import time

comms = serial.Serial('/dev/ttyUSB0', baudrate=57600)

while True:
    recv = comms.read_all()

    if len(recv) > 0:
        print(recv)

    time.sleep(0.05)
