import pygame, pigame
from pygame.locals import *
import os
import sys
from time import sleep
import RPi.GPIO as GPIO
import time
import serial
import csv
from gpiozero import MCP3008
from datetime import datetime

# Colours
WHITE = (255, 255, 255)

# Setup touchscreen
os.putenv('SDL_VIDEODRV', 'fbcon')
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV', 'dummy')
os.putenv('SDL_MOUSEDEV', '/dev/null')
os.putenv('DISPLAY', '')

# Setting up serial communication
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial OK")

# Setup pitft screen
pygame.init()
pitft = pigame.PiTft()
lcd = pygame.display.set_mode((320, 240))
lcd.fill((0, 0, 0))
pygame.display.update()

# Initialize callback for bail-out button
not_ended = True
def quit_callback(channel):
    global not_ended
    not_ended = False

# Initialize variables for two_collide
size = width, height = 320, 240
speed = [3, 3]
speed2 = [1, -1]
black = 0, 0, 0

# Initialize GPIO callback
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(27, GPIO.FALLING, quit_callback)

# Initialize font and buttons
font_big = pygame.font.Font(None, 30)
touch_buttons = {
    'Start': (30, 220), 'Stop': (280, 220), 'Osc. Test': (160, 220),
    'Frequency:': (60, 20), '1': (140, 20), '0.832': (200, 20), '0.719': (290, 20),
    'WEC:': (30, 80), 'Osc. Flap': (120, 80), 'Point Abs.': (260, 80),
    'Amp:': (30, 120), '0.02': (90, 120), '0.06': (160, 120), '0.1': (240, 120)
}

# Initialize variables to see if two collide is being displayed
started = False
continued = False
stopped = False

# Initialize clock
clock = pygame.time.Clock()
FPS = 30
clock.tick(FPS)
pygame.display.update()

# Define functions for sending commands to the serial port
def send_test_command(frequency, wec_type, amp):
    message = f"Frequency:{frequency}|Type:{wec_type}|Amp:{amp}\n"
    ser.write(message.encode('utf-8'))

def send_command(frequency, wec_type):
    message = f"Frequency:{frequency}|Type:{wec_type}\n"
    ser.write(message.encode('utf-8'))

# Initialize variables for frequency, wec_type, and amp
frequency = 1
wec_type = "OS"
amp = 0.02

# Create log filename
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f'adc_readings_{current_time}.csv'

# Initialize the MCP3008 ADC
adc = MCP3008(channel=0)

# Start data logging when "Osc. Test" is pressed
logging_started = False
with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'ADC Value'])

    try:
        while not_ended:
            pitft.update()
            clock.tick(FPS)

            # Scan touchscreen events
            for event in pygame.event.get():
                sleep(0.1)
                clock.tick(FPS)
                if event.type == MOUSEBUTTONDOWN:
                    x, y = pygame.mouse.get_pos()
                    print(x, y)
                elif event.type == MOUSEBUTTONUP:
                    # Update display with the right buttons
                    lcd.fill((0, 0, 0))
                    for k, v in touch_buttons.items():
                        text_surface = font_big.render(f'{k}', True, WHITE)
                        rect = text_surface.get_rect(center=v)
                        lcd.blit(text_surface, rect)

                    x, y = pygame.mouse.get_pos()

                    # Handle Start, Stop, and Test button presses
                    if x > 280 and y > 210:
                        stopped = True
                        started = False
                        logging_started = False  # Stop logging
                    elif x < 30 and y > 210:
                        started = True
                        stopped = False
                        logging_started = False  # Stop logging
                    elif 130 < x < 210 and y > 210:
                        print("Test")
                        continued = True
                        stopped = False
                        send_test_command(frequency, wec_type, amp)
                    # Frequency selection
                    elif 130 < x < 200 and y < 30:
                        print("Frequency 1")
                        frequency = 1
                    elif 210 < x < 280 and y < 30:
                        print("Frequency 0.832")
                        frequency = 0.832
                    elif x > 296 and y < 30:
                        print("Frequency 0.719")
                        frequency = 0.719
                    # WEC type selection
                    elif 90 < x < 160 and 60 < y < 90:
                        print("Osc WEC")
                        wec_type = "OS"
                    elif 230 < x and 60 < y < 90:
                        print("Point Abs")
                        wec_type = "PA"
                    # Amplitude selection
                    elif 80 < x < 130 and y < 130:
                        print("Amp 0.02")
                        amp = 0.02
                    elif 150 < x < 210 and y < 130:
                        print("Amp 0.06")
                        amp = 0.06
                    elif x > 230 and y < 130:
                        print("Amp 0.1")
                        amp = 0.1

                    # Start and Stop button handling
                    if started and not continued:
                        print("Started")
                        send_command(frequency, wec_type)
                        continued = True
                        logging_started = True  # Start logging
                    elif stopped and continued:
                        print("Stopped")
                        ser.write("Stop\n".encode('utf-8'))
                        continued = False
                        logging_started = False  # Stop logging

                    print(x, y)

            # Start logging if the button is pressed
            if logging_started:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                value = adc.value
                writer.writerow([timestamp, value])
                print(f"Logging ADC value: {value} at {timestamp}")

            sleep(0.000001)
    
    except KeyboardInterrupt:
        pass

    finally:
        print(f"Program finished. Data logged to {log_filename}.")
        del pitft
