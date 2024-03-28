from commands2 import Command
from constants import *
from subsystems.lift import Lift, LiftStates
from subsystems.swerve import Swerve
from typing import Callable
from wpimath.filter import SlewRateLimiter

class ControlSwerveSpeed(Command):
    speed_controller = SlewRateLimiter(2.5, -2.5)
    
    def __init__(self, lift: Lift, swerve: Swerve, override: Callable[[None],bool]):
        super().__init__()
        
        self.lift = lift
        self.swerve = swerve
        self.override = override
        
        # We don't add the requirements to this since we aren't actually "messing with" 
        # either mechanisms, but instead reading values to setting the swerve speed.
        
        self.desired_speed = SwerveConstants.k_max_module_speed
        
    def initialize(self):
        pass
    
    def execute(self):
        if self.override():
            self.desired_speed = SwerveConstants.k_max_module_speed
            self.swerve.set_max_module_speed(self.desired_speed)
            return

        if self.lift.getState() is LiftStates.RAISED or self.lift.getState() is LiftStates.CONTROLLED:
            self.desired_speed = self.speed_controller.calculate(SwerveConstants.k_max_module_speed / 3.5)
            self.swerve.set_max_module_speed(self.desired_speed)
            self.swerve.set_module_override_brake(False)
        elif self.lift.getState() is LiftStates.SCORE:
            self.desired_speed = self.speed_controller.calculate(SwerveConstants.k_max_module_speed / 1.5)
            self.swerve.set_max_module_speed(self.desired_speed)
            self.swerve.set_module_override_brake(True)
        else:
            self.desired_speed = self.speed_controller.calculate(SwerveConstants.k_max_module_speed)
            self.swerve.set_max_module_speed(self.desired_speed)
            self.swerve.set_module_override_brake(True)
    
    def end(self, interrupted: bool):
        pass
    
    def isFinished(self) -> bool:
        return False