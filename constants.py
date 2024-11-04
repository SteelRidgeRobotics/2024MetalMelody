from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue, TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from pathplannerlib.controller import PIDConstants
from phoenix6.configs import Slot0Configs
from phoenix6.configs.talon_fx_configs import *
from wpilib import RobotBase
from wpimath.geometry import Translation2d

class DrivetrainConstants:

    k_canbus_name = "rio" # The name of the CAN bus with all the swerve devices. Use "rio" if not using a CANivore

    k_steer_gains = Slot0Configs() \
    .with_k_p(100).with_k_i(0).with_k_d(0.2) \
    .with_k_s(0).with_k_v(1.5).with_k_a(0)

    k_drive_gains = Slot0Configs() \
    .with_k_p(0.35).with_k_i(0).with_k_d(0) \
    .with_k_s(0).with_k_v(1.2).with_k_a(0)

    # Stator current at which the wheels start to slip.
    k_slip_current = 80

    k_max_rot_rate = 5.607 # Max chassis rotation rate (rad/s)
    k_max_drive_speed = 4.712389 # Max speed of the robot (m/s)

    k_wheel_diameter = 0.1 # meters

    k_drive_gear_ratio = 27 / 4
    k_steer_gear_ratio = 150 / 7

    k_module_locations = (
        Translation2d(0.587, 0.587), # Left front
        Translation2d(-0.587, 0.587), # Left rear
        Translation2d(0.587, -0.587), # Right front
        Translation2d(-0.587, -0.587) # Right rear
    )

    # Encoder Offsets
    k_left_front_offset = 0.474853515625
    k_left_rear_offset = -0.0009765625
    k_right_front_offset = 0.398681640625
    k_right_rear_offset = -0.41845703125

    k_is_pigeon_gyro = True or not RobotBase.isReal() # If False, uses NavX. Simultaion uses the Pigeon sim_state for control.

class Auto:
    k_drive_base_radius = 0.83 # Radius from center of robot to swerve modules in meters
    k_translation_pid = PIDConstants(5, 0, 0, 0)
    k_rotation_pid = PIDConstants(5, 0, 0, 0)
        
class Limelight:
        
    k_enable_vision_odometry = RobotBase.isReal() # False if there's no Limelight on the robot.
        
    k_limelight_name = "limelight" # "limelight" by default. Name of the limelight to use for vision.

    k_use_mega_tag_2 = True # If False, uses MegaTag 1.
        
    k_standard_deviations = [0.3, 0.3, 99999] # (x, y, radians) Basically how confident we are with our vision, lower = more confident. Angle is set really high because we have a gyro.

class CanIDs:

    k_left_front_drive = 1
    k_left_rear_drive = 2
    k_right_front_drive = 3
    k_right_rear_drive = 4
        
    k_left_front_direction = 5
    k_left_rear_direction = 6
    k_right_front_direction = 7
    k_right_rear_direction = 8
        
    # CANcoders
    k_left_front_encoder = 5
    k_left_rear_encoder = 6
    k_right_front_encoder = 7
    k_right_rear_encoder = 8

    # Pigeon
    k_pigeon_gyro = 9
    
    PIVOTMOTOR = 9
    INTAKEMOTOR = 10
    
    LIFTMOTOR_RIGHT = 11
    LIFTMOTOR_LEFT = 12

class IntakeConstants:
    GEAR_RATIO = 5
    INTAKESPEED = 1

class PivotConstants:
    MM_ACCELERATION = 4
    MM_CRUISE_VEL = 0.5
    MM_JERK = 20
    STOWPOS = 0
    INTAKEPOS = 0.361
    SCOREPOSDOWN = 0.268
    SCOREPOSUP = 0.107 #For scoring up into the amp (yell at Kaylee not me ;-;) 

    K_P = 10
    K_I = 0
    K_D = 0.2
    K_V = 0.12
    K_S = 0.2
    GEAR_RATIO = 50
    SUPPLY_LIMIT = 5

class LiftConstants:
    CURRENTSUPPLYLIMIT = 25
    TOPPOSITION = 78.635 # lol big number
    SCOREPOSITION = 22.254
    BOTTOMPOSITION = 0
    GEARRATIO = 12
    K_P = 1
    K_I = 0.1
    MM_ACCEL = 75
    MM_VEL = 100

class ExternalConstants:
    DRIVERCONTROLLER = 0
    FUNCTIONSCONTROLLER = 1
    TRIGGER_DEADBAND = 0.1
