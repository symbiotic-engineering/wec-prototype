#!/usr/bin/env python 3
import serial
import time
import RPi.GPIO as GPIO

#set pin mode to broadcom
GPIO.setmode(GPIO.BCM)

#set GPIO
GPIO.setup(17,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(22,GPIO.IN, pull_up_down=GPIO.PUD_UP)


#Setting up serial comm
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial OK")

#callback functions for commands
def start_callback(channel):
    time.sleep(1)
    print("Send message")
    ser.write("Osc\n".encode('utf-8'))
    
def stop_callback(channel):
    time.sleep(1)
    print("Send message")
    ser.write("PA\n".encode('utf-8'))

try:
    GPIO.add_event_detect(17, GPIO.FALLING,start_callback)
    GPIO.add_event_detect(22, GPIO.FALLING,stop_callback)

except KeyboardInterrupt:
    print("Close serial comm")
    ser.close()      
