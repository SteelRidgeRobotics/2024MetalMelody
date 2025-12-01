package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private TalonFX masterMotor = new TalonFX(frc.robot.Constants.CanIDs.LEFT_PIVOT_TALON);
  private TalonFX followerMotor = new TalonFX(frc.robot.Constants.CanIDs.RIGHT_PIVOT_TALON);
  private MotionMagicDutyCycle motionMagicControl;

  public Pivot() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.RotorToSensorRatio = Constants.PivotConstants.GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = Constants.PivotConstants.GAINS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.MM_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.MM_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = Constants.PivotConstants.MM_CRUISE_VELOCITY * 0.2;

    masterMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);

    Follower followerRequest = new Follower(frc.robot.Constants.CanIDs.LEFT_PIVOT_TALON, true);
    followerMotor.setControl(followerRequest);

    masterMotor.setPosition(0);
  }

  public void setPosition(double setpoint) {
    
    masterMotor.setControl(motionMagicControl.withPosition(setpoint));
  }
}
