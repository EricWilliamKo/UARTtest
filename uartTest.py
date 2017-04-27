#!/usr/bin/env python
import serial
import time

megaSerial = serial.Serial("/dev/mega_wheel", 115200)

while True:
    serin = 0
    serin = megaSerial.read(megaSerial.inWaiting())
    print str(serin)
    if megaSerial.in_waiting == 0:
        megaSerial.write('what the fuck 1234567890987654321 yo yo yo yo yeah yo motherfucker hello rugby123 what the fuck 1234567890987654321 yo yo yo yo yeah yo motherfucker hello rugby123 ')
        time.sleep(.01)
