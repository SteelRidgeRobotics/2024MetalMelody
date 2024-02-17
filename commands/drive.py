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
    NORMAL = 0
    AMP = 1

class DriveByController(Command):

    def __init__(self, camera: Camera, swerve: Swerve, controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.camera = camera
        self.addRequirements(self.swerve)

        self.controller = controller
        self.mode = DriveModes.NORMAL
        self.camera.setPipeline(0)
    
    def execute(self) -> None:
        translation_x = (-deadband(self.controller.getLeftY(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        translation_y = (-deadband(self.controller.getLeftX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-deadband(self.controller.getRightX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_rot_rate
        
        slowdown_mult = 1
        if self.controller.getLeftBumper():
            slowdown_mult += 0.5
        if self.controller.getRightBumper():
            slowdown_mult += 0.5
            
        if self.controller.getAButton():
                self.swerve.pivot_around_point(rotation * SwerveConstants.k_max_rot_rate / slowdown_mult, Translation2d(3.56, Rotation2d()))
                return
        
        inputted_speeds = ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult)
        if rotation == 0:
            pass
            #inputted_speeds = self.swerve.convert_to_angle_lock(inputted_speeds)
        else:
            self.swerve.reset_desired_heading()
        
        if self.mode == DriveModes.AMP and self.camera.getTagId() == self.getAmpTagID():
            x_diff, unused_hi_ally = self.camera.getDistanceToTag()
            self.swerve.field_relative_drive(ChassisSpeeds(inputted_speeds.vx, -x_diff * DriveConstants.translation_kP, inputted_speeds.omega))
        else:
            self.swerve.field_relative_drive(inputted_speeds)
            
        # Toggle Modes
        if self.controller.getYButtonPressed():
            if self.mode == DriveModes.NORMAL:
                self.mode = DriveModes.AMP
                self.camera.setPipeline(0)
            else:
                self.mode = DriveModes.NORMAL
                self.camera.setPipeline(1)
                self.swerve.set_desired_heading(self.getAmpAngleTarget().degrees())
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
    def getAmpAngleTarget() -> Rotation2d:
        return Rotation2d.fromDegrees(-90) if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else Rotation2d.fromDegrees(90)
    
    def getAmpTagID() -> int:
        return 6 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else 5
    
def deadband(value: float, band: float):
    if fabs(value) <= band:
        return 0
    else:
        return value
