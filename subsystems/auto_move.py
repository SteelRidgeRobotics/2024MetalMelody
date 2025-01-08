from commands2 import Subsystem
import ntcore
from limelight import LimelightHelpers
from constants import Constants
import math
from wpilib import DriverStation
import wpilib


class AutoMove(Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Automove")
        self.ntInstance = ntcore.NetworkTableInstance.getDefault()

    def periodic(self) -> None:
        self.calculateDistance()

    def calculateDistance(self, april_tag_id):
        robot_pose = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("").pose.translation()
        tag_pose = Constants.k_apriltag_layout.getTagPose(april_tag_id).toPose2d().translation()
        robot_to_tag_dist = math.sqrt(math.fabs(robot_pose.x-tag_pose.x)**2+math.fabs(robot_pose.y-tag_pose.y)**2)
        
