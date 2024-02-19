from math import fabs, pi, sqrt

from commands2 import InstantCommand, Subsystem
import navx
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
from phoenix6 import BaseStatusSignal
from phoenix6.configs.cancoder_configs import *
from phoenix6.configs.talon_fx_configs import *
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.controls import *
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.signals import *
from typing import Self
from wpilib import DriverStation, Field2d, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState

from constants import *


class SwerveModule(Subsystem):


    def __init__(self, module_name: str, drive_motor_constants: DriveMotorConstants, direction_motor_constants: DirectionMotorConstants, CANcoder_id: int, CAN_offset: float) -> None:
        super().__init__()

        self.module_name = module_name

        self.turning_encoder = CANcoder(CANcoder_id, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE).with_magnet_offset(CAN_offset).with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
        self.turning_encoder.configurator.apply(encoder_config)
        self.turning_encoder.optimize_bus_utilization() # Will likely turn the encoder into a SyncedCANcoder in the future, but for now we literally only get its pos on startup.
        
        self.drive_motor = TalonFX(drive_motor_constants.motor_id, "rio")
        drive_motor_constants.apply_configuration(self.drive_motor)
        
        BaseStatusSignal.set_update_frequency_for_all(250, self.drive_motor.get_velocity(), self.drive_motor.get_position(), self.drive_motor.get_motor_voltage())
        self.drive_motor.optimize_bus_utilization()

        self.direction_motor = TalonFX(direction_motor_constants.motor_id, "rio")
        direction_motor_constants.apply_configuration(self.direction_motor)
        
        BaseStatusSignal.set_update_frequency_for_all(250, self.direction_motor.get_rotor_position())
        self.direction_motor.optimize_bus_utilization()
        
        self.directionTargetPos = self.directionTargetAngle = 0.0

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(rots_to_degs(self.direction_motor.get_rotor_position().value / k_direction_gear_ratio))
    
    def reset_sensor_position(self) -> None:
        self.direction_motor.set_position(-self.turning_encoder.get_absolute_position().wait_for_update(0.02).value * k_direction_gear_ratio)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(rots_to_meters(self.drive_motor.get_velocity().value), self.get_angle())
    
    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(rots_to_meters(self.drive_motor.get_position().value), self.get_angle())

    def set_desired_state(self, desiredState: SwerveModuleState, override_brake_dur_neutral: bool=True) -> None:
        desiredState.optimize(desiredState, self.get_angle())
        desiredAngle = desiredState.angle.degrees() % 360

        angleDist = fabs(desiredAngle - self.directionTargetAngle)

        if (angleDist > 90 and angleDist < 270):
            targetAngle = (desiredAngle + 180) % 360
            self.invert_factor = -1
        else:
            targetAngle = desiredAngle
            self.invert_factor = 1

        targetAngleDist = fabs(targetAngle - self.directionTargetAngle)

        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        changeInRots = degs_to_rots(targetAngleDist)

        angleDiff = targetAngle - self.directionTargetAngle

        if angleDiff < 0:
            angleDiff += 360

        if angleDiff > 180:
            self.directionTargetPos -= changeInRots
        else:
            self.directionTargetPos += changeInRots

        self.directionTargetAngle = targetAngle

        self.direction_motor.set_control(MotionMagicVoltage(self.directionTargetPos * k_direction_gear_ratio))
        self.drive_motor.set_control(VelocityVoltage(meters_to_rots(self.invert_factor * desiredState.speed, k_drive_gear_ratio), override_brake_dur_neutral=override_brake_dur_neutral))
       

class Swerve(Subsystem):
    navx = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1), Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    left_front: SwerveModule = SwerveModule("LF", DriveMotorConstants(MotorIDs.LEFT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_FRONT_DIRECTION), CANIDs.LEFT_FRONT, -0.77001953125)
    left_rear: SwerveModule = SwerveModule("LR", DriveMotorConstants(MotorIDs.LEFT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_REAR_DIRECTION), CANIDs.LEFT_REAR, -0.49951171875)
    right_front: SwerveModule = SwerveModule("RF", DriveMotorConstants(MotorIDs.RIGHT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_FRONT_DIRECTION), CANIDs.RIGHT_FRONT, -0.430419921875)
    right_rear: SwerveModule = SwerveModule("RR", DriveMotorConstants(MotorIDs.RIGHT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_REAR_DIRECTION), CANIDs.RIGHT_REAR, -0.403564453125)
    
    
    def __init__(self):
        super().__init__()

        self.odometry = SwerveDrive4PoseEstimator(self.kinematics, self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), Pose2d())

        SmartDashboard.putData(self.field)
        reset_yaw = InstantCommand(lambda: self.reset_yaw())
        reset_yaw.setName("Reset Yaw")
        SmartDashboard.putData("Reset Gyro", reset_yaw)
        
        self.set_max_module_speed()
        
        if not AutoBuilder.isConfigured():
            AutoBuilder.configureHolonomic(
                lambda: self.get_pose(),
                lambda pose: self.reset_odometry(pose),
                lambda: self.get_robot_relative_speeds(),
                lambda chassisSpeed: self.robot_centric_drive(chassisSpeed),
                HolonomicPathFollowerConfig(
                    PIDConstants(SwerveConstants.auto_kP_translation, 0.0, 0.0, 0.0), # translation
                    PIDConstants(SwerveConstants.auto_kP_rotation, 0.0, 0.0, 0.0), # rotation
                    SwerveConstants.k_max_module_speed,
                    SwerveConstants.k_drive_base_radius,
                    ReplanningConfig()
                ),
                lambda: self.should_flip_auto_path(),
                self
            )
        
        self.navx.reset()
        self.desired_heading = 0
        self.obdn = True

    def should_flip_auto_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.navx.getYaw())
    
    def field_relative_drive(self, chassis_speed: ChassisSpeeds, center_of_rotation: Translation2d=Translation2d()) -> None: # Discretizes the chassis speeds, then transforms it into individual swerve module states (field relative)
        self.set_module_states(self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds.discretize(chassis_speed, 0.02), self.get_angle()), centerOfRotation=center_of_rotation))
        
    def robot_centric_drive(self, chassis_speed: ChassisSpeeds, center_of_rotation: Translation2d=Translation2d()) -> None: # see drive(), but less cool to watch
        self.set_module_states(self.kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassis_speed, 0.02)))
        
    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds((self.left_front.get_state(), self.left_rear.get_state(), self.right_front.get_state(), self.right_rear.get_state()))

    def set_module_states(self, module_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(module_states, self.max_module_speed)

        self.left_front.set_desired_state(desatStates[0], override_brake_dur_neutral=self.obdn)
        self.left_rear.set_desired_state(desatStates[1], override_brake_dur_neutral=self.obdn)
        self.right_front.set_desired_state(desatStates[2], override_brake_dur_neutral=self.obdn)
        self.right_rear.set_desired_state(desatStates[3], override_brake_dur_neutral=self.obdn)
        
    def set_max_module_speed(self, max_module_speed: float=SwerveConstants.k_max_module_speed) -> None:
        self.max_module_speed = max_module_speed
        
    def set_module_override_brake(self, new_obdn: bool) -> None:
        self.obdn = new_obdn

    def set_voltage(self, volts: float) -> None:
        """For SysId tuning"""
        self.left_front.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=self.obdn))
        self.left_rear.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=self.obdn))
        self.right_front.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=self.obdn))
        self.right_rear.drive_motor.set_control(VoltageOut(volts, override_brake_dur_neutral=self.obdn))
        
    def log_motor_output(self, log: SysIdRoutineLog) -> None: # Unsued since we just convert the hoot file
        pass

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def reset_odometry(self, pose=Pose2d()) -> None:
        self.odometry.resetPosition(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), pose)
    
    def reset_yaw(self) -> Self:
        self.navx.reset()
        return self

    def periodic(self) -> None:
        self.field.setRobotPose(self.odometry.update(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position())))
        SmartDashboard.putData(self.field)

    def addVisionMeasurement(self, pose: Pose2d, timestamp: float) -> None:
        current_pose = self.odometry.getEstimatedPosition()
        if sqrt((current_pose.X() - pose.X())**2 + (current_pose.Y() - pose.Y())**2) > 1:
            return
        self.odometry.addVisionMeasurement(pose, timestamp)
        
    def initialize(self) -> None:
        self.left_front.reset_sensor_position()
        self.left_rear.reset_sensor_position()
        self.right_front.reset_sensor_position()
        self.right_rear.reset_sensor_position()

"""
CONVERSIONS
"""

def meters_to_rots(meters: float, ratio: float) -> float:
    return meters / (pi * SwerveConstants.k_wheel_size) * ratio

def rots_to_meters(rotation: float, ratio: float=1) -> float:
    return (rotation / ratio) * (pi * SwerveConstants.k_wheel_size)

def rots_to_degs(rotation: float) -> float:
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    return degrees / 360
