import math

import navx
from commands2 import Command, Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (HolonomicPathFollowerConfig, PIDConstants,
                                   ReplanningConfig)
from phoenix6.configs.cancoder_configs import *
from phoenix6.configs.talon_fx_configs import *
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.controls import *
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.signals import *
from typing import Self
from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import (ChassisSpeeds, SwerveDrive4Kinematics,
                                SwerveModulePosition,
                                SwerveModuleState)

from constants import *


class SwerveModule(Subsystem):
    
    def __init__(self, module_name: str, drive_motor_constants: DriveMotorConstants, direction_motor_constants: DirectionMotorConstants, CANcoder_id: int, CAN_offset: float) -> None:
        super().__init__()

        self.module_name = module_name

        self.turning_encoder = CANcoder(CANcoder_id, "rio")
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE).with_magnet_offset(CAN_offset).with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
        self.turning_encoder.configurator.apply(encoder_config)
        
        self.drive_motor = TalonFX(drive_motor_constants.motor_id, "rio")
        drive_motor_constants.apply_configuration(self.drive_motor)

        self.direction_motor = TalonFX(direction_motor_constants.motor_id, "rio")
        direction_motor_constants.apply_configuration(self.direction_motor)
        
        self.directionTargetPos = self.directionTargetAngle = 0.0

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(rots_to_degs(self.direction_motor.get_rotor_position().value / k_direction_gear_ratio))
    
    def reset_sensor_position(self) -> None:
        pos = -self.turning_encoder.get_absolute_position().value
        self.direction_motor.set_position(pos * k_direction_gear_ratio)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(rots_to_meters(self.drive_motor.get_rotor_velocity().value, k_drive_gear_ratio), self.get_angle())
    
    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(rots_to_meters(self.drive_motor.get_rotor_position().value, k_drive_gear_ratio), self.get_angle())

    def set_desired_state(self, desiredState: SwerveModuleState) -> None:
        desiredState.optimize(desiredState, self.get_angle())
        desiredAngle = desiredState.angle.degrees() % 360

        angleDist = math.fabs(desiredAngle - self.directionTargetAngle)

        if angleDist in range(91, 270):
            targetAngle = (desiredAngle + 180) % 360
            invert_factor = -1
        else:
            targetAngle = desiredAngle
            invert_factor = 1

        targetAngleDist = math.fabs(targetAngle - self.directionTargetAngle)

        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        angleDiff = targetAngle - self.directionTargetAngle

        while angleDiff < 0:
            angleDiff += 360

        if angleDiff > 180:
            self.directionTargetPos -= degs_to_rots(targetAngleDist)
        else:
            self.directionTargetPos += degs_to_rots(targetAngleDist)

        self.directionTargetAngle = targetAngle

        self.direction_motor.set_control(MotionMagicVoltage(self.directionTargetPos * k_direction_gear_ratio))
        self.drive_motor.set_control(VelocityVoltage(meters_to_rots(invert_factor * desiredState.speed, k_drive_gear_ratio)))
       

class Swerve(Subsystem):
    navx = navx.AHRS.create_spi()

    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1),
                                        Translation2d(1, -1), Translation2d(-1, -1)) # LF, LR, RF, RR
    
    field = Field2d()
    
    left_front: SwerveModule = SwerveModule("LF", DriveMotorConstants(MotorIDs.LEFT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_FRONT_DIRECTION), CANIDs.LEFT_FRONT, -0.473388671875)
    left_rear: SwerveModule = SwerveModule("LR", DriveMotorConstants(MotorIDs.LEFT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_REAR_DIRECTION), CANIDs.LEFT_REAR, -0.9990234375)
    right_front: SwerveModule = SwerveModule("RF", DriveMotorConstants(MotorIDs.RIGHT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_FRONT_DIRECTION), CANIDs.RIGHT_FRONT, -0.39990234375)
    right_rear: SwerveModule = SwerveModule("RR", DriveMotorConstants(MotorIDs.RIGHT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_REAR_DIRECTION), CANIDs.RIGHT_REAR, 0.41943359375)
    
    def __init__(self):
        super().__init__()

        self.odometry = SwerveDrive4PoseEstimator(self.kinematics, self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), Pose2d())

        SmartDashboard.putData(self.field)
        SmartDashboard.putData("Reset Odometry", self.reset_odometry_command())
        SmartDashboard.putData("Reset Gyro", self.reset_gyro_command())
        
        AutoBuilder.configureHolonomic(
            lambda: self.get_pose(),
            lambda pose: self.reset_odometry(pose),
            lambda: self.get_chassis_speeds(),
            lambda chassisSpeed: self.drive(chassisSpeed, field_relative=False),
            HolonomicPathFollowerConfig(
                PIDConstants(1.5, 0.0, 0.0, 0.0), # translation
                PIDConstants(2.5, 0.0, 0.0, 0.0), # rotation
                SwerveConstants.k_max_module_speed,
                SwerveConstants.k_drive_base_radius,
                ReplanningConfig()
            ),
            lambda: self.should_flip_auto_path(),
            self
        )
        
        self.navx.reset()

    def should_flip_auto_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.navx.getYaw())
    
    def drive(self, chassis_speed: ChassisSpeeds, field_relative: bool=True):
        chassis_speed = ChassisSpeeds.discretize(chassis_speed, 0.02)
        if field_relative:
            states = self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassis_speed, self.get_angle()))
        else:
            states = self.kinematics.toSwerveModuleStates(chassis_speed)

        desat_states = self.kinematics.desaturateWheelSpeeds(states, SwerveConstants.k_max_module_speed)

        self.set_module_states(desat_states)

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds((self.left_front.get_state(), self.left_rear.get_state(), self.right_front.get_state(), self.right_rear.get_state()))

    def set_module_states(self, module_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        desatStates = self.kinematics.desaturateWheelSpeeds(module_states, SwerveConstants.k_max_module_speed)

        self.left_front.set_desired_state(desatStates[0])
        self.left_rear.set_desired_state(desatStates[1])
        self.right_front.set_desired_state(desatStates[2])
        self.right_rear.set_desired_state(desatStates[3])

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def reset_odometry(self, pose=Pose2d()) -> None:
        self.odometry.resetPosition(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position()), pose)
        
    def reset_odometry_command(self) -> Command:
        return self.runOnce(lambda: self.reset_odometry())
    
    def reset_gyro_command(self) -> Command:
        return self.runOnce(lambda: self.reset_yaw())
    
    def reset_yaw(self) -> Self:
        self.navx.reset()
        return self

    def periodic(self) -> None:
        self.field.setRobotPose(self.odometry.update(self.get_angle(), (self.left_front.get_position(), self.left_rear.get_position(), self.right_front.get_position(), self.right_rear.get_position())))
        SmartDashboard.putData(self.field)
        
    def initialize(self):
        self.left_front.reset_sensor_position()
        self.left_rear.reset_sensor_position()
        self.right_front.reset_sensor_position()
        self.right_rear.reset_sensor_position()

"""Conversions"""

def meters_to_rots(meters: float, ratio: float) -> float:
    return (meters / (math.pi * SwerveConstants.k_wheel_size)) * ratio

def rots_to_meters(rotation: float, ratio: float=1) -> float:
    return (rotation / ratio) * (math.pi * SwerveConstants.k_wheel_size)

def rots_to_degs(rotation: float) -> float:
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    return degrees / 360
