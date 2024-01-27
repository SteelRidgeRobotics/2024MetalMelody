from enum import Enum

class MotorIDs(Enum):
    PIVOTMOTOR = 0
    INTAKEMOTOR = 1
    ELEVATORMOTOR = 2

class MotorConstants(Enum):
    PIVOTSPEED = .5
    INTAKESPEED = .8
    ELEVATORTOGGLE = True
    TOPPOSITION = 0
    BOTTOMPOSITION = 0