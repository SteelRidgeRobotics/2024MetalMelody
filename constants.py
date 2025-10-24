from phoenix6.configs.config_groups import Slot0Configs
from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class Constants:

    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

    class CanIDs:
        LEFT_ELEVATOR_TALON = 10
        RIGHT_ELEVATOR_TALON = 11

        INTAKE_TALON = 12
        INTAKE_TALON2 = 13
        ALGAE_TALON = 14
        
        LEFT_PIVOT_TALON = 16
        RIGHT_PIVOT_TALON = 17

        ELEVATOR_CANDI = 20
        PIVOT_CANCODER = 21

        INTAKE_CANRANGE = 23

    class ElevatorConstants:
        L1_SCORE_POSITION = 75
        L2_SCORE_POSITION = 60
        L3_SCORE_POSITION = 60 # We will not be able to reach this position, so we will not use it.
        L2_ALGAE_POSITION = 60
        L3_ALGAE_POSITION = 60 # We will not be able to reach this position, so we will not use it.
        PROCESSOR_SCORE_POSITION = 75
        CAM_POSITION = 100.620117
        ELEVATOR_MAX = 52.342773

        DEFAULT_POSITION = 101.472656

        CRUISE_VELOCITY = 100
        MM_JERK = len("העלא")*1000000
        MM_UPWARD_ACCELERATION = 75
        MM_BRAKE_ACCELERATION = 24
        MM_DOWNWARD_ACCELERATION = 12

        GEAR_RATIO = 12/1  #Pulled from old code
        GAINS = (Slot0Configs()
            .with_k_g(0.36)
            .with_k_p(1)
            .with_k_i(0.1)
            .with_k_d(0.0)
            .with_k_s(0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )
        SETPOINT_TOLERANCE = 0.1

    class PivotConstants:
        # INSIDE_ELEVATOR_ANGLE = 0.2 # Used for subsystem collision checking
        #ELEVATOR_PRIORITY_ANGLE = 0.123535 # We move the pivot to this position until the elevator has reached its setpoint.
        START_ANGLE = -0.024902
        STOW_ANGLE = -0.510254
        GROUND_INTAKE_ANGLE = -10.788086
        ALGAE_INTAKE_ANGLE = -10.788086
        HIGH_SCORING_ANGLE =  -10.788086
        MID_SCORING_ANGLE = -10.788086
        LOW_SCORING_ANGLE = -10.788086
        NET_SCORING_ANGLE = -10.788086
        PROCESSOR_SCORING_ANGLE = -10.788086
        # CLIMBER_PRIORITY_ANGLE = 0.201943 -1

        MINIMUM_ANGLE = -10.788086
        MAXIMUM_ANGLE = -0.024902

        CRUISE_VELOCITY = 3
        MM_ACCELERATION = 3

        GEAR_RATIO = 961/36 # TODO: Verify this gear ratio.
        GAINS = (Slot0Configs()
                 .with_k_g(0.27)
                 .with_k_p(30)
                 .with_k_i(0.0)
                 .with_k_d(0.6343)
                 .with_k_s(0.19)
                 .with_k_v(0)
                 .with_k_a(0)
                 .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        CANCODER_DISCONTINUITY = 0.5
        CANCODER_OFFSET = -0.434326171875 

        SETPOINT_TOLERANCE = 0.03125

    class IntakeConstants:
        # TODO: Verify these speeds.
        CORAL_INTAKE_SPEED = 0.4*1.2*1.1
        CORAL_OUTPUT_SPEED = 0.6
        L1_OUTPUT_SPEED = -0.4

        ALGAE_HOLD = 0.125
        ALGAE_INTAKE_SPEED = 0.75
        ALGAE_OUTPUT_SPEED = -1

        SUPPLY_CURRENT = 35

        GEAR_RATIO = 4
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

    class VisionConstants:
        FRONT_LEFT = "limelight-fl"
        FRONT_RIGHT = "limelight-fr"
        FRONT_CENTER = "limelight-front"
        BACK_CENTER = "limelight-back"
    
    class AutoAlignConstants:

        MAX_DISTANCE = 3.6343
        
        TRANSLATION_P = 12
        TRANSLATION_I = 0
        TRANSLATION_D = 0.1
        
        HEADING_P = 2
        HEADING_I = 0
        HEADING_D = 0.2
        
        HEADING_TOLERANCE = 2

        VELOCITY_DEADBAND = 0.1
        ROTATIONAL_DEADBAND = 0.02