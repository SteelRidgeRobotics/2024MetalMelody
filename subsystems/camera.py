from commands2 import Subsystem
import ntcore
from wpimath.geometry import Pose2d, Rotation2d

class Camera(Subsystem):

    def __init__(self) -> None:
        super().__init__()
        self.limelight = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        
        self.fieldPose = [0, 0, 0, 0, 0, 0, 0]
        self.currentTag = 0
        self.offsetX = self.offsetY = 0
        self.totalLatency = 0
        
        self.setPipeline(0)

    def getField2dPose(self) -> Pose2d:
        return Pose2d(self.fieldPose[0], self.fieldPose[1], Rotation2d.fromDegrees(self.fieldPose[5]))
    
    def getDistanceToTag(self) -> tuple[float, float]:
        return (self.offsetX, self.offsetY)
    
    def getTotalLatency(self) -> float:
        return self.totalLatency
    
    def getTagId(self) -> int:
        return self.currentTag
    
    def periodic(self) -> None:
        self.currentTag = self.limelight.getEntry("tid").getInteger(0)
        if self.currentTag == -1:
            self.fieldPose = [0, 0, 0, 0, 0, 0, 0]
            self.offsetX = 0
            self.offsetY = 0
            return
        self.fieldPose = self.limelight.getEntry("botpose_wpiblue").getDoubleArray([0, 0, 0, 0, 0, 0, 0])
        self.offsetX = self.limelight.getEntry("tx").getDouble(0)
        self.offsetY = self.limelight.getEntry("ty").getDouble(0)
        self.totalLatency = self.limelight.getEntry("tl").getDouble(0) + self.limelight.getEntry("cl").getDouble(0)
        
    def setPipeline(self, id: int) -> None:
        self.limelight.putNumber("pipeline", id)
        
    def getPipeline(self) -> int:
        return self.limelight.getEntry("pipeline").getDouble(0)
