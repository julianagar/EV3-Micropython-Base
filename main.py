#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import challengeTemplate

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
chainSaw = Motor(Port.A)
touchSensor1 = TouchSensor(Port.S1)
touchSensor2 = TouchSensor(Port.S2)
ultraSensor = UltrasonicSensor(Port.S4) 

# Initialize the drive base. 
# MIGHT WANT TO CHECK TO MAKE SURE THIS IS RIGHT
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
robot.settings(10000, 10000, 7000, 7000)

ev3.speaker.beep()
ev3.screen.print(UltrasonicSensor.distance(silent=False))
ev3.screen.print(touchSensor)

while touchSensor.pressed() == False:
    while UltrasonicSensor.distance() == 0:
        robot.turn(-180)
        robot.turn(180)
        if UltrasonicSensor.distance() > 0: 
            turnAngle = UltrasonicSensor.distance()
            ev3.screen.print(touchSensor())

    while touchSensor2.pressed() == False:
        robot.turn(turnAngle)
        robot.drive(1000)
        if touchSensor2.pressed():
            break

    
    
