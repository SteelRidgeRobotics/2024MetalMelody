from enum import Enum

class MotorIDs:
    PIVOTMOTOR = 0
    INTAKEMOTOR = 1
    ELEVATORMOTOR = 6

class IntakeConstants:
    PIVOTSPEED = .5
    INTAKESPEED = .2

class ElevatorConstants:
    
    TOPPOSITION = -140
    BOTTOMPOSITION = 17
    ELEVATORLENGTH = 158.135
    MIDDLEPOSITION = (TOPPOSITION + BOTTOMPOSITION)/2
    FEEDFORWARD = 0 # change this for whatever the feedforward will be
    kP = 1
    MOTIONMAGICACCELERATION = 40
    MOTIONMAGICVELOCITY = 60
    MOTIONMAGICJERK = 240

class ExternalConstants:
    DRIVERCONTROLLER = 0
    FUNCTIONSCONTROLLER = 1
