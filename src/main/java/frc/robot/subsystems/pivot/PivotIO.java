package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIO {
        public boolean pivotConnected = false;
        public double positionRads = 0.0;
        public double velocityRads = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void setPivotOpenLoop(double output) {}

    default void setPivotPosition(double position) {}
}
