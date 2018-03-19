import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import pickle
import os
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import time
import select
import RPi.GPIO as GPIO
import pigpio

# Setup UDP Stuff
UDP_PORT =5004
SIZE=1024

hostname = gethostbyname( '0.0.0.0' )
sock = socket(AF_INET, SOCK_DGRAM)
sock.bind((hostname, UDP_PORT))
sock.setblocking(0)
# Setup servo motor and dc motors
# Define pins used.
servoPin = 24
pi = pigpio.pi()


PIN = 18
PWMA1 = 6
PWMA2 = 13
PWMB1 = 20
PWMB2 = 21
PWMC1 = 24
D1 = 12
D2 = 26

PWM = 50

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PIN,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(PWMA1,GPIO.OUT)
GPIO.setup(PWMA2,GPIO.OUT)
GPIO.setup(PWMB1,GPIO.OUT)
GPIO.setup(PWMB2,GPIO.OUT)
GPIO.setup(PWMC1,GPIO.OUT)
GPIO.setup(D1,GPIO.OUT)
GPIO.setup(D2,GPIO.OUT)
p1 = GPIO.PWM(D1,500)
p2 = GPIO.PWM(D2,500)
p3 = GPIO.PWM(PWMC1,50)
p1.start(0)
p2.start(0)

#Sets motor output based on four input values through outputting to GPIO pins.


def set_motor(A1, A2, B1, B2):
    GPIO.output(PWMA1, A1)
    GPIO.output(PWMA2, A2)
    GPIO.output(PWMB1, B1)
    GPIO.output(PWMB2, B2)

# Forward involves pushing both motors forward.


def forward():
    # PWMA1 and PWMB1 set high.
    set_motor(1, 0, 1, 0)


def stop():
	set_motor(0,0,0,0)

def reverse():
    #PWMA2 and PWMB2 set high.
	set_motor(0,1,0,1)

def left():
    #PWMA1 and PWMB2 set high.
	set_motor(1,0,0,0)

def right():
    #PWMA2 and PWMB1 set high.
	set_motor(0,0,1,0)

# This function will take a float from -1 to 1 and map it to a degree from 0 to 180

def floattodeg (num):
    # Only allow angle to go from 30 to 150
    # Chassis is blocking wheels from turning anymore
    return (num * 60) + 90

# Expects an angle between -60 and 60

def setAngle(angle):
    if (angle < -60):
        angle = -60
    elif (angle > 60):
        angle = 60
    angle = angle + 90
    #print("Angle: ", angle)
    pulseWidth = angle*5.55555555555555555555555555555555555556 + 987
    #print("pulseWidth: ", pulseWidth)
    pi.set_servo_pulsewidth(servoPin,pulseWidth)
    time.sleep(0.001)

def setSpeed(speed):
    reverse()
    p1.ChangeDutyCycle(speed)
    p1.ChangeDutyCycle(speed)



# -------- Main Program Loop -----------
# Make Car go forward at constant speed
reverse()
PWM = 40 # Defines the speed (ranges from 0 to 100)
p1.ChangeDutyCycle(PWM)
p2.ChangeDutyCycle(PWM)
timeout_in_seconds = 1
while True:
    for i in range(5):
        reverse()
        p1.ChangeDutyCycle(PWM)
        p2.ChangeDutyCycle(PWM)
        # If time it takes to receive an angle goes over 1 seconds, kill program
        #t1 =time.time()
        ready = select.select([sock], [], [], timeout_in_seconds)
        stop_flag = 0
        if ready[0]:

            #Convert byte to string 
            data, addr = sock.recvfrom(1024)
            data_string = str(data, 'utf-8')

            #if string is a number string or a bool, 
            if data_string == 'True':
                stop_flag = 1
                time.sleep(2)
                stop_flag = 0

            else:
                steering_angle = float(data_string)
                setAngle(steering) 
        else:
            reverse()
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(0)
            stop()
            break
        #t2 = time.time()
        #print("deta time: ", t2-t1)
 

          


# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
#forward()
#p1.ChangeDutyCycle(0)
#p2.ChangeDutyCycle()
#stop()
