from commands2 import Command
from subsystems.intake import Intake
from subsystems.elevator import Elevator
from subsystems.camera import Camera
from subsystems.swerve import Swerve
from wpimath.kinematics import ChassisSpeeds

class LineUpToAmp(Command):

    def __init__(self, camera: Camera, swerve: Swerve):

        self.camera = camera
        self.swerve = swerve

        self.kP = 0.01

        self.addRequirements(self.swerve)

    def execute(self) -> None:

        if self.camera.getTagId() == 6:

            self.offsetX, self.offsetY = self.camera.getDistanceToTag()
            adjust = self.offsetX * self.kP
            self.swerve.robot_centric_drive(ChassisSpeeds(0, -adjust))

            # if not lined up, return
            # move forward
            # if past needed position, stop
            # end command

    def isFinished(self):

        return self.camera.getTagId() == 6 and self.offsetX == 0 and self.offsetY == 0 #offsets to be determined
    
    def end(self):

        # stop robot











        
    



    