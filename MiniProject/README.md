# Mini Project

## Objective
#### Design a control system that regualtes the rotational speed of a wheel attached to a motor using an Arduino and Raspberry Pi. The wheel is marked along the circumference with numbers respresenting the angle in radians. Using a single Aruco marker, direct the system to spin the wheel so that the wheel is at one of the four angles indicated on the wheel. The control system will reject disturbances, so that if someone tries to spin the wheel, the motor will resist and the marker will remain at the top. The system should be fast enough that all four markers can be moved to the top one after the other in less than 10 seconds.

## Organization

#### Computer Vision:
##### Use Open CV to implement the Aruco Detection Scheme

#### Systems Integration:
##### Setup communication between the Raspberry Pi, Arduino and LCD

#### Control Systems:
##### Create a simulation of your motor and design a controller for your motor

#### Localizaiton:
##### Use a motor driver to spin a motor under control of the Arduino and use the arduino to read the rotary encoder on the motor
