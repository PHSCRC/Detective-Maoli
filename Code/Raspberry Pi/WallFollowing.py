import time
import serial
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

OK_DISTANCE = 5
ACCEPTABLE_DIF = 0.05
TOO_FAR = 10000000
# put this in later after testing
# acceptable difference before it's clear that the robot is skewed relative to the wall


def percentError(int1, int2):
    return abs(int1 - int2)/((int1+int2)/2)
    
# def takeReadingForROS():
    
# def takeReadingForIRL():
    

def align():
    reading = scanPort.read()
    if reading[0] > reading[1]:
        while not (percentError(reading[0], reading[1]) < ACCEPTABLE_DIF):
            turn(-0.1)
            reading = scanPort.read()
    else:
        while not (percentError(reading[0], reading[1]) < ACCEPTABLE_DIF):
            turn(-0.1)
            reading = scanPort.read()


def approachFlame():
    reading = scanPort.read()
    while reading[4] != "close enough":
        move(0.01)
        reading = scanPort.read()


def move(distance):
    scanPort.write("Move:", distance)


def turn(degrees):
    scanPort.write("Turn:", degrees)


def turnRight():
    move(0.05)
    turn(90)


def turnLeft():
    move(0.05)
    turn(-90)


def main():
    global scanPort 
    scanPort = serial.Serial(
        port=' /dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)
        
    global motorPort 
    motorPort = serial.Serial(
        port=' /dev/ttyACM0',
        # replace tty once you know what the port which is using the motor arduino is called

        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)
    time.sleep(1)
        
    while True:
        reading = scanPort.read()
        # pass me a tuple/array of 3 elements (first two are the side sensors, last element is the front sensor reading) 
        if percentError(reading[0], reading[1]) > ACCEPTABLE_DIF or reading[2] < OK_DISTANCE or reading[0] >= TOO_FAR or reading[4] == "flame":
            if reading[4] == "flame":
                approachFlame()
                scanPort.write("KILL THE FLAME")
            if percentError(reading[0], reading[1]) > ACCEPTABLE_DIF & reading[0] >= TOO_FAR:
                turnRight()
            else:
                align()
            if reading[2] > OK_DISTANCE & reading[0] < TOO_FAR:
                turnLeft()
        else:
            move(0.05)
            
        
main()
