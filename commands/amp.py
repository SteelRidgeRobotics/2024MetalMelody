from commands2 import Command
from subsystems.camera import Camera
from subsystems.swerve import Swerve
from wpilib import DriverStation, XboxController
from wpimath.kinematics import ChassisSpeeds

class LineUpToAmp(Command):
    
    def __init__(self, camera: Camera, swerve: Swerve, controller: XboxController) -> None:
        self.swerve = swerve
        self.camera = camera
        self.addRequirements(self.swerve, self.camera)
        
        self.controller = controller
        
        self.k_p = -0.05
        
    def initialize(self):
        self.errorX = 0
        self.errorY = 0
        
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.targetID = 6
        else:
            self.targetID = 5
    
    def execute(self):
        if self.camera.getTagId() == self.targetID:
            dx, dy = self.camera.getDistanceToTag()
            
            speedX = self.k_p * dx
            speedY = self.k_p * dy
            
            self.swerve.drive(ChassisSpeeds(speedX, speedY), field_relative=False)
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.swerve.drive(ChassisSpeeds())
    
    def isFinished(self) -> bool:
        return self.controller.getBButton()
