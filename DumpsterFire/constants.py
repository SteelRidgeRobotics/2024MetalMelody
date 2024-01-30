from enum import Enum

class MotorIDs(Enum):
    PIVOTMOTOR = 0
    INTAKEMOTOR = 1
    ELEVATORMOTOR = 2

class MotorConstants(Enum):
    PIVOTSPEED = .5
    INTAKESPEED = .8
    ELEVATORTOGGLE = True
    TOPPOSITION = 0 # make this a range for the top position
    BOTTOMPOSITION = 0 # make this a rannge for the bottom position
    FEEDFORWARD = 0 # change this for whatever the feedforward will be