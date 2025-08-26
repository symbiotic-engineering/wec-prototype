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
font_big = pygame.font.Font(None, 30)
touch_buttons = {'Start':(30,220),'Stop':(280,220), 'Osc. Test': (160,220), 'Frequency:': (60,20), '1':(140,20), '0.832':(200,20), '0.719':(290,20),
                 'WEC:': (30,80), 'Osc. Flap': (120, 80), 'Point Abs.':(260,80),
                 'Amp:': (30,120), '0.02': (90, 120), '0.06':(160,120), '0.1':(240,120)}
for k,v in touch_buttons.items():
    text_surface = font_big.render('%s'%k, True, WHITE)
    rect = text_surface.get_rect(center=v)
    lcd.blit(text_surface, rect)

#initialize variable to see if two collide is being displayed
started= False
continued = False
stopped=False

clock = pygame.time.Clock()
FPS = 30
clock.tick(FPS)
pygame.display.update()

def send_test_command(frequency, wec_type, amp):
    message = f"Frequency:{frequency}|Type:{wec_type}|Amp:{amp}\n"
    ser.write(message.encode('utf-8'))

def send_command(frequency, wec_type):
    message = f"Frequency:{frequency}|Type:{wec_type}\n"
    ser.write(message.encode('utf-8'))
    
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
                
                #Start, Stop and Test touch detection  
                if(x>280 and y>210):
                    stopped=True
                    started=False
                elif (x<30 and y>210):
                    started=True
                    stopped=False
                elif(130<x<210 and y>210):
                    print("Test")
                    continued = True
                    stopped = False
                    send_test_command(frequency, wec_type, amp)
                
                #Frequency touch detection 
                elif(130<x<200 and y<30):
                    print("Frequency 1")
                    frequency = 1
                elif(210<x<280 and y<30):
                    print("Frequency 0.832")
                    frequency = 0.832
                elif(x>296 and y<30):
                    print("Frequency 0.719")
                    frequency = 0.719
                
                #WEC type selection
                elif(90<x<160 and 60<y<90):
                    print("Osc WEC")
                    wec_type = "OS"
                elif(230<x and 60<y<90):
                    print("Point Abs")
                    wec_type = "PA"  
                    
                #Amplitude detection   
                elif(80<x<130 and y<130):
                    print("amp 0.02")
                    amp = 0.02
                elif(150<x<210 and y<130):
                    print("amp 0.06")
                    amp = 0.06
                elif(x>230 and y<130):
                    print("amp 0.1")
                    amp = 0.1
                
                #Start and Stop
                if(started and not continued):
                    print("Started")
                    send_command(frequency, wec_type)
                    continued = True
                elif(stopped and continued):
                    print("Stopped")
                    ser.write("Stop\n".encode('utf-8'))
                    continued = False
                    
                print(x,y)
                        
        #yield the thread for callbacks 
        sleep(0.000001)
except KeyboardInterrupt:
    pass
finally:
    del(pitft)
