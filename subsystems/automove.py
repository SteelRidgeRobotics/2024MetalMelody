from commands2 import Subsystem
import ntcore
from limelight import LimelightHelpers

class Automove(Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Automove")
        self.ntInstance = ntcore.NetworkTableInstance.getDefault()

    def periodic(self) -> None:
        self.calculateDistance()

    def calculateDistance(self):
        robot_pose = LimelightHelpers.