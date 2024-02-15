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
        
        if self.mode == DriveModes.NORMAL:
            self.swerve.field_relative_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, rotation / slowdown_mult))
            
        else:
            angle_dist = fabs(self.getAmpAngleTarget().radians() - self.swerve.get_angle().radians())
            
            adjust = DriveConstants.rotation_kP * angle_dist
            
            angle_diff = self.getAmpAngleTarget().degrees() - self.swerve.get_angle().degrees()
            if angle_diff < 0:
                angle_diff += 360
            if angle_diff > 180:
                adjust *= -1
            
            if self.camera.getTagId() == self.getAmpTagID():
                # align ourselves to all wyatt has to do is **drive forward** (forward being to the amp)
                x_diff, unused_hi_ally = self.camera.getDistanceToTag()
                self.swerve.field_relative_drive(ChassisSpeeds(translation_x / slowdown_mult, -x_diff * DriveConstants.translation_kP, adjust))
            else:
                self.swerve.field_relative_drive(ChassisSpeeds(translation_x / slowdown_mult, translation_y / slowdown_mult, adjust))
            
        # Toggle Modes
        if self.controller.getYButtonPressed():
            if self.mode == DriveModes.NORMAL:
                self.mode = DriveModes.AMP
                self.camera.setPipeline(0)
            else:
                self.mode = DriveModes.NORMAL
                self.camera.setPipeline(1)
    
    def end(self, interrupted: bool) -> None:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False
    
    def getAmpAngleTarget(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-90) if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else Rotation2d.fromDegrees(90)
    
    def getAmpTagID(self) -> int:
        return 6 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else 5
    
def deadband(value: float, band: float):
    if fabs(value) <= band:
        return 0
    else:
        return value
