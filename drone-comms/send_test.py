#!/bin/python3

import serial

comms = serial.Serial('/dev/ttyUSB0', baudrate=57600)
comms.write(b'Hello drone!\r\n')

comms.close()
