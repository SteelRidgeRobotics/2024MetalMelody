from commands2 import Command
from constants import *
from frc6343.controller.deadband import deadband
from phoenix6.controls import VoltageOut
from subsystems.elevator import Elevator
from wpilib import XboxController

class ManualElevator(Command):

    def __init__(self, controller: XboxController, elevator: Elevator):
        super().__init__()

        self.controller = controller
        self.elevator = elevator

        self.addRequirements(self.elevator)

    def execute(self):
        self.elevator.setDutyCycle(VoltageOut(self.getTriggerCombinedValue() * 6))

    def end(self, interrupted: bool):
        self.elevator.setDutyCycle(VoltageOut(0))

    def getTriggerCombinedValue(self) -> float:
        return -deadband(self.controller.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND)