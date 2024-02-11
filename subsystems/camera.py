from commands2 import Subsystem
import ntcore
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

class Camera(Subsystem):

    def __init__(self) -> None:
        super().__init__()

        self.calculatedXY = Translation2d()

        self.nt = ntcore.NetworkTableInstance.getDefault()

    def getCalculatedXY(self) -> Translation2d:
        return self.calculatedXY

    def periodic(self) -> None:
        limelight = self.nt.getTable("limelight")
        tx = limelight.getEntry("tx").getDouble(0)
        ty = limelight.getEntry("ty").getDouble(0)

        self.calculatedXY = Translation2d.fromFeet(tx, ty)
    