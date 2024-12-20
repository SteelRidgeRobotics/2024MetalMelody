from commands2 import Subsystem
from constants import Constants
import ntcore
import math
from limelight import LimelightHelpers
from subsystems.swerve import SwerveSubsystem
from wpimath.kinematics import ChassisSpeeds
from wpilib import DriverStation
import wpilib
import robotpy_apriltag

class VisionSubsystem(Subsystem):

    def __init__(self):
        self.ntInstance = ntcore.NetworkTableInstance.getDefault()

    def periodic(self) -> None:
        self.calculateDegrees()

    def calculateDegrees(self):

        # read from LimeLight
        # know which alliance
        # confirm correct tag
        # calculate the angle to speaker
        # ! calculate the hypot for distance
        # convert degrees to rotations per constant of gear ratio
        # return number of rotations
        # ! calculate the rev speed
        
        table = self.ntInstance.getTable("limelight")
        # NetworkTables.getTable("limelight").putNumber('priorityid',4) I have a topic on this pending in Chief Delphi so I'll know how to write this.
        targetOffsetAngle = table.getNumber("ty",0.0)
        tagId = table.getNumber("tid", 0)
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.ntInstance.getTable("limelight").putNumber('priorityid', Constants.LimeLight.REDSPEAKERID)

            wpilib.SmartDashboard.putString("ID Priority", "Red (4)")
            if tagId != Constants.LimeLight.REDSPEAKERID: #need ids for blue or red and also check that
                return 0
            

        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.ntInstance.getTable("limelight").putNumber('priorityid', Constants.LimeLight.BLUESPEAKERID)
            wpilib.SmartDashboard.putString("ID Priority", "Blue (7)")

            if tagId != Constants.LimeLight.BLUESPEAKERID: #need ids for blue or red and also check that
                return 0
        
        angleToTargetRadians = self.getAngleToTargetInRadians(targetOffsetAngle)
        distanceToGoal = self.getDistanceToTargetInches(4)
        degrees = self.getDegreesToSpeaker(distanceToGoal)
        rotations = (degrees * Constants.Swivel.GEAR_RATIO) / 360

        wpilib.SmartDashboard.putNumber("rotations", rotations)
        return rotations

    def getAngleToTargetInRadians(self, targetOffsetAngle):
        angleToGoalDegrees = Constants.LimeLight.k_mount_angle  + targetOffsetAngle
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0)

        wpilib.SmartDashboard.putNumber("angle to target (rad)", angleToGoalRadians)
        return angleToGoalRadians
    
    # Would only be used if we want to adjust velocity of launcher or determine if we are too close or far
    def getDistanceToTargetInches(self, april_tag_id: int):

        robot_pose = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("").pose.translation()

        tag_pose = Constants.k_apriltag_layout.getTagPose(april_tag_id).toPose2d().translation()

        robot_to_tag_dist = math.sqrt(math.fabs(robot_pose.x-tag_pose.x)**2+math.fabs(robot_pose.y-tag_pose.y)**2)

        rttd_inches = robot_to_tag_dist*39.37

        #distanceToTargetInches=(Constants.LimeLight.k_tag_height - Constants.LimeLight.k_mount_height)/math.tan(angleToTargetRadians)

        wpilib.SmartDashboard.putNumber("D", rttd_inches)
        return rttd_inches
    
    def getDegreesToSpeaker(self, distance):
        if distance <= 0.0:
            return Constants.Swivel.MAX_ANGLE
        degrees = math.degrees(math.atan(Constants.LimeLight.k_target_height/distance))
        if degrees > Constants.Swivel.MAX_ANGLE:
            return Constants.Swivel.MAX_ANGLE
        
        wpilib.SmartDashboard.putNumber("Degrees to speaker", degrees)
        return degrees
