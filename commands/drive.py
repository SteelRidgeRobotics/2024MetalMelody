from commands2 import Command
from constants import *
from math import fabs
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

class DriveByController(Command):

    def __init__(self, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.addRequirements(self.swerve)

        self.controller = controller

    def initialize(self) -> None:
        self.swerve.initialize()        
    
    def execute(self) -> None:
        translation_x = self.controller.getLeftY()
        translation_y = self.controller.getLeftX()
        rotation = self.controller.getRightX()
        
        # Bumper Slowdown
        slowdown_mult = 1
        if self.controller.getLeftBumper():
            slowdown_mult += 0.5
        if self.controller.getRightBumper():
            slowdown_mult += 0.5

        translation_y = -deadband(translation_y, ExternalConstants.DEADBAND) ** 3
        translation_x = -deadband(translation_x, ExternalConstants.DEADBAND) ** 3
        rotation = -deadband(rotation, ExternalConstants.DEADBAND) ** 3
        
        if self.controller.getAButton():
            self.swerve.pivot_around_point(rotation * SwerveConstants.k_max_rot_rate / slowdown_mult, Translation2d(3.56, Rotation2d()))
            return

        self.swerve.drive(ChassisSpeeds(translation_x * SwerveConstants.k_max_module_speed / slowdown_mult, 
                                        translation_y * SwerveConstants.k_max_module_speed / slowdown_mult, 
                                        rotation * SwerveConstants.k_max_rot_rate / slowdown_mult), field_relative=True)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
def deadband(value: float, band: float):
    """
    value is the value we want to deadband
    the band is the abs value the value can not be less than
    """
    # this makes sure that joystick drifting is not an issue.
    # It takes the small values and forces it to be zero if smaller than the 
    # band value
    if fabs(value) <= band:
        return 0
    else:
        return value
