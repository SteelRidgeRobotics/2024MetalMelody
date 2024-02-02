import commands2
from constants import *
from subsystems.elevator import Elevator
import phoenix6


class MoveElevator(commands2.Command):

    def __init__(self, elevator: Elevator):

        self.elevator = elevator
        
        self.addRequirements(self.elevator)

    def execute(self):

        if self.elevator.isDown:

            self.elevator.move(ElevatorConstants.BOTTOMPOSITION)

        else:

            self.elevator.move(ElevatorConstants.TOPPOSITION)
            
    def isFinished(self):

        return False