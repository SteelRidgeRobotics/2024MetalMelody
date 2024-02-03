from enum import Enum

class MotorIDs:
    PIVOTMOTOR = 0
    INTAKEMOTOR = 1
    ELEVATORMOTOR = 6

class IntakeConstants:
    PIVOTSPEED = .5
    INTAKESPEED = .2

class ElevatorConstants:
    
    TOPPOSITION = 0 # make this a range for the top position
    BOTTOMPOSITION = 106.269 # make this a rannge for the bottom position
    FEEDFORWARD = -51.866 # change this for whatever the feedforward will be
    kP = 1
    MOTIONMAGICACCELERATION = 120
    MOTIONMAGICVELOCITY = 120
    MOTIONMAGICJERK = 250

class ExternalConstants:
    DRIVERCONTROLLER = 0
    FUNCTIONSCONTROLLER = 1
