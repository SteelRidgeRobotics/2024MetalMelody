from commands2 import Command
from constants import *
from enum import Enum
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.kinematics import ChassisSpeeds

class DriveModes(Enum):
    FIELD_RELATIVE = 0
    ROBOT_CENTRIC = 1

class DriveByController(Command):
    def __init__(self, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.addRequirements(self.swerve)

        self.controller = controller
        self.mode = DriveModes.FIELD_RELATIVE
    
    def execute(self) -> None:
        translation_x = (-self.controller.getLeftY() ** 3) * SwerveConstants.k_max_module_speed
        translation_y = (-self.controller.getLeftX() ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-self.controller.getRightX() ** 3) * SwerveConstants.k_max_rot_rate
        
        slowdown_mult = 1
        if self.controller.getRightBumper():
            slowdown_mult += 1
        
        if self.mode == DriveModes.FIELD_RELATIVE:
            self.swerve.field_relative_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult))
        else:
            self.swerve.robot_centric_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult))
            
        # Toggle Modes
        if self.controller.getYButtonPressed():
            if self.mode == DriveModes.FIELD_RELATIVE:
                self.mode = DriveModes.ROBOT_CENTRIC
            else:
                self.mode = DriveModes.FIELD_RELATIVE
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
