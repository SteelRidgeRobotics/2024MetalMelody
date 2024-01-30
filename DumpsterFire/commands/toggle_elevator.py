import typing
import commands2
from constants import *
from subsystems.elevator import Elevator


class ToggleElevator(commands2.Command):

    def __init__(self, elevator: Elevator):

        self.elevator = elevator
        
        self.addRequirements([self.elevator])

    def execute(self):

        if self.elevator.isExtended:
            self.elevator.move(MotorConstants.BOTTOMPOSITION)
        elif not self.elevator.isExtended:
            self.elevator.move(MotorConstants.TOPPOSITION)
    
    def isFinished(self):
        
        self.elevatorPos = self.elevator.elevatorMotor.get_rotor_position().value

        if self.elevator.isExtended and abs(MotorConstants.BOTTOMPOSITION - self.elevatorPos) <= 3:

            return True

        elif not self.elevator.isExtended and abs(MotorConstants.TOPPOSITION - self.elevatorPos) <= 3:

            return True
        
        else:

            return False

         

