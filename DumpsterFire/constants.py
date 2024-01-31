from enum import Enum

class MotorIDs(Enum):
    PIVOTMOTOR = 0
    INTAKEMOTOR = 1

class MotorConstants(Enum):
    PIVOTSPEED = .5
    INTAKESPEED = .8