from commands2 import Command
from constants import *
from frc6343.controller.deadband import deadband
from phoenix6.controls import DutyCycleOut
from subsystems.elevator import Elevator
from wpilib import XboxController

class ManualElevator(Command):
    
    def __init__(self, elevator: Elevator, controller: XboxController):
        super().__init__()
        
        self.elevator = elevator
        self.addRequirements(self.elevator)
        
        self.controller = controller
        
    def initialize(self):
        self.elevator.setDutyCycle(DutyCycleOut(0))
        
    def execute(self):
       self.elevator.setDutyCycle(deadband(-self.controller.getLeftY() , ExternalConstants.DEADBAND) ** 3 * 0.25)
       
    def isFinished(self) -> bool:
        return self.controller.getBButtonReleased()
    
    def end(self, interrupted: bool):
        self.elevator.setDutyCycle(0)
