import commands2
from constants import *
from subsystems.elevator import Elevator


class ToggleElevator(commands2.Command):

    def __init__(self, elevator: Elevator):

        self.elevator = elevator
        
        self.addRequirements(self.elevator)

    def execute(self):

        self.elevator.isDown = not self.elevator.isDown
    
    def isFinished(self):
        
        return True

         

