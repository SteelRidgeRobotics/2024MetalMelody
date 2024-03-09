from commands2 import Command
from constants import *
from frc6343.controller.deadband import deadband
from phoenix6.controls import TorqueCurrentFOC
from subsystems.lift import Lift
from wpilib import XboxController

class ManualLift(Command):

    def __init__(self, controller: XboxController, lift: Lift):
        super().__init__()

        self.controller = controller
        self.lift = lift

        self.addRequirements(self.lift)

    def execute(self):
        self.lift.master_motor.set_control(TorqueCurrentFOC(-self.getTriggerCombinedValue() * 100, limit_forward_motion=True))

    def end(self, interrupted: bool):
        self.lift.stop()

    def getTriggerCombinedValue(self) -> float:
        return deadband(self.controller.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND)