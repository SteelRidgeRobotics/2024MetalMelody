from commands2 import Subsystem
import ntcore
from wpimath.geometry import Pose2d, Rotation2d

class Camera(Subsystem):
    """
    periodic scans network tables for any apriltags found.
    """

    def __init__(self) -> None:
        super().__init__()
        self.limelight = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        
        self.fieldPose = [0, 0, 0, 0, 0, 0, 0]
        self.currentTag = 0
        self.offsetX = self.offsetY = 0

    def getField2dPose(self) -> Pose2d:
        return Pose2d(self.fieldPose[0], self.fieldPose[1], Rotation2d.fromDegrees(self.fieldPose[5]))
    
    def getDistanceToTag(self) -> tuple[float, float]:
        return (self.offsetX, self.offsetY)
    
    def periodic(self) -> None:
        self.fieldPose = self.limelight.getEntry("botpose_wpiblue").getDoubleArray([0, 0, 0, 0, 0, 0, 0])
        self.currentTag = self.limelight.getEntry("tid").getInteger(0)
        self.offsetX = self.limelight.getEntry("tx").getDouble(0)
        self.offsetY = self.limelight.getEntry("ty").getDouble(0)
