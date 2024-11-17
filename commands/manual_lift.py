from commands2 import Command
from constants import Constants
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
    
    def initialize(self):
        self.lift.activateFollower()

    def execute(self):
        trigger_value = self.getTriggerCombinedValue()
        if trigger_value == 0:
            self.lift.stop()
        else:
            self.lift.setControl(TorqueCurrentFOC(-325, max_abs_duty_cycle=self.getTriggerCombinedValue(), limit_forward_motion=True))

    def end(self, interrupted: bool):
        self.lift.stop()

    def getTriggerCombinedValue(self) -> float:
        return deadband(self.controller.getLeftTriggerAxis(), 0.1)