# Python script to test sendning waveshare JSON command to the 
# base robot.
# Install pyserial with apt get install pyserial
# json should be already included with Pyhton install
# Author: Ziad Youssfi, April, 2025

import serial
import json
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
print(ser.name)
print(ser.baudrate)
print(ser.bytesize)
if(ser.is_open):
    print('serial is open')

# Form the JSON commnad string
json_string = '{"T":13,"X":0.0,"Z":2.9}'
json_string = json_string + '\n'
print(json_string.encode("utf-8"))

# Send the JSON string
written = ser.write(json_string.encode("utf-8"))
print(written)
ser.flush()

# Wait a bit before shutting the motors
time.sleep(2)
ser.write((json.dumps('{"T":1,"L":0,"R":0}') + '\n').encode("utf-8"))

# Wait a bit before exiting the script to allow time for the last command
time.sleep(1)

