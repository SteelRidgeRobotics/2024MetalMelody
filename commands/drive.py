from commands2 import Command
from constants import *
from enum import Enum
from math import fabs
from subsystems.camera import Camera
from subsystems.swerve import Swerve
from wpilib import DriverStation, XboxController
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

class DriveModes(Enum):
    FIELD_RELATIVE = 0
    ROBOT_CENTRIC = 1

class DriveByController(Command):
    def __init__(self, camera: Camera, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.camera = camera
        self.addRequirements(self.swerve)

        self.controller = controller
        self.mode = DriveModes.FIELD_RELATIVE
    
    def execute(self) -> None:
        translation_x = (-deadband(self.controller.getLeftY(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        translation_y = (-deadband(self.controller.getLeftX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-deadband(self.controller.getRightX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_rot_rate
        
        slowdown_mult = 1
        if self.controller.getRightBumper():
            slowdown_mult += 1
            
        if self.controller.getLeftBumper():
            center_of_rotation = Translation2d(3.56, 0)
        else:
            center_of_rotation = Translation2d()
        
        if self.mode == DriveModes.FIELD_RELATIVE:
            self.swerve.field_relative_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult), center_of_rotation=center_of_rotation)
        else:
            self.swerve.robot_centric_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult), center_of_rotation=center_of_rotation)
            
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
    
def deadband(value: float, band: float):
    if fabs(value) <= band:
        return 0
    else:
        return value
