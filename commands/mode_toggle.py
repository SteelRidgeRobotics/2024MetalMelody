from commands2 import Command
from constants import *
from wpilib import SmartDashboard
from typing import Callable

class ModeToggle(Command):

    mode = Modes.INTAKE

    def __init__(self):
        
        super().__init__()

    def execute(self):

        ModeToggle.mode = (ModeToggle.mode + 1) % 2

    def isFinished(self):
        return True

    @staticmethod
    def get_mode():
        return ModeToggle.mode
    

    
    







