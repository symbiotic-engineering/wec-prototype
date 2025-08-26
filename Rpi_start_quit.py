// By: Yashaswini Mandalam

import pygame,pigame
from pygame.locals import *
import os
import sys
from time import sleep
import RPi.GPIO as GPIO
import time
import serial
#Colours
WHITE = (255,255,255)

#setup touchscreen
os.putenv('SDL_VIDEODRV','fbcon')
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV','dummy')
os.putenv('SDL_MOUSEDEV','/dev/null')
os.putenv('DISPLAY','')

#Setting up serial comm
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
time.sleep(3)
ser.reset_input_buffer()
print("Serial OK")

#setup pitft screen
pygame.init()
pitft = pigame.PiTft()
lcd = pygame.display.set_mode((320, 240))
lcd.fill((0,0,0))
pygame.display.update()

#initialize callback for bail out button
not_ended=True
def quit_callback(channel):
    global not_ended
    not_ended=False
   
#initialize variables for two_collide
size = width, height  =320,240
speed =[3,3]
speed2 =[1,-1]
black =0,0,0

#initialize GPIO callback
GPIO.setup(27,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(27, GPIO.FALLING,quit_callback)
     
#intialize font, and sets of buttons to be displayed on the screen
font_big = pygame.font.Font(None, 20)
touch_buttons = {'Start':(20,220),'Quit':(290,220)}
for k,v in touch_buttons.items():
    text_surface = font_big.render('%s'%k, True, WHITE)
    rect = text_surface.get_rect(center=v)
    lcd.blit(text_surface, rect)

#initialize variable to see if two collide is being displayed
started= False
continued = False
#initialize pause variable
stopped=False

clock = pygame.time.Clock()
FPS = 30
clock.tick(FPS)
pygame.display.update()
   
#initialize start time
start_time=time.time()
try:
    #automatically quit if the it has been 60 seconds or bail out button pressed
    while not_ended:
        pitft.update()
        #update framerate
        clock.tick(FPS)
         
        # Scan touchscreen events
        for event in pygame.event.get():
            sleep(0.1)
            clock.tick(FPS)
            if(event.type is MOUSEBUTTONDOWN):
                x,y = pygame.mouse.get_pos()
                print(x,y)
            elif(event.type is MOUSEBUTTONUP):
                #update display with the right buttons
                lcd.fill((0,0,0))
                for k,v in touch_buttons.items():
                    text_surface = font_big.render('%s'%k, True, WHITE)
                    rect = text_surface.get_rect(center=v)
                    lcd.blit(text_surface, rect)
                x,y = pygame.mouse.get_pos()
                #if not started update the location per button press
                if(stopped):
                    ser.write("Stop\n".encode('utf-8'))
                #display two_collide if the variable is right
       
                if(x>280 and y>210):
                    ser.write("Stop\n".encode('utf-8'))
                    started = False;
                    continued = False;
                elif (x<30 and y>210):
                    started=True
               
                if(started and not continued):
                    print("Started")
                    ser.write("Start\n".encode('utf-8'))
                    continued = True
                   
                print(x,y)
                       
        #yield the thread for callbacks
        sleep(0.000001)
except KeyboardInterrupt:
    pass
finally:
    del(pitft)
