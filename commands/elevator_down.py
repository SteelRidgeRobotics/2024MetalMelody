import wpilib
import commands2
from subsystems.elevator import Elevator

class ElevatorDown(commands2.Command):

    def __init__(self, elevator: Elevator):

        self.elevator = elevator

    def execute(self):

        self.elevator.stage = 0
        
    

