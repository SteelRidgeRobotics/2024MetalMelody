from commands2 import Subsystem
from limelight import LimelightHelpers
from navx import AHRS
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from pathplannerlib.config import HolonomicPathFollowerConfig
from pathplannerlib.logging import PathPlannerLogging
from phoenix6.hardware import Pigeon2
from phoenix6.status_signal import BaseStatusSignal
from wpilib import DriverStation, Field2d, RobotBase, RobotController, SmartDashboard, Timer
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState

from constants import Constants
from util import *
from subsystems.drive.swerve_module import SwerveModule


class Drivetrain(Subsystem):
    """Swerve drivetrain :partying_face:"""
    
    # Creating all modules (in a tuple for organization)
    modules = (
        SwerveModule(
            Constants.CanIDs.k_left_front_drive,
            Constants.CanIDs.k_left_front_direction,
            Constants.CanIDs.k_left_front_encoder,
            Constants.DrivetrainConstants.k_left_front_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_left_rear_drive,
            Constants.CanIDs.k_left_rear_direction,
            Constants.CanIDs.k_left_rear_encoder,
            Constants.DrivetrainConstants.k_left_rear_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_right_front_drive,
            Constants.CanIDs.k_right_front_direction,
            Constants.CanIDs.k_right_front_encoder,
            Constants.DrivetrainConstants.k_right_front_offset
        ),
        SwerveModule(
            Constants.CanIDs.k_right_rear_drive,
            Constants.CanIDs.k_right_rear_direction,
            Constants.CanIDs.k_right_rear_encoder,
            Constants.DrivetrainConstants.k_right_rear_offset
        )
    )

    # Kinematics
    kinematics = SwerveDrive4Kinematics(
        Constants.DrivetrainConstants.k_module_locations[0], 
        Constants.DrivetrainConstants.k_module_locations[1],
        Constants.DrivetrainConstants.k_module_locations[2], 
        Constants.DrivetrainConstants.k_module_locations[3]
    )

    # PathPlanner Config
    path_follower_config = HolonomicPathFollowerConfig(
        Constants.AutoConstants.k_translation_pid, 
        Constants.AutoConstants.k_rotation_pid, 
        Constants.DrivetrainConstants.k_max_drive_speed,
        Constants.AutoConstants.k_drive_base_radius,
        ReplanningConfig()
    )

    def __init__(self, starting_pose: Pose2d = Pose2d()) -> None:
        
        self.gyro: Pigeon2 | AHRS = None
        if Constants.DrivetrainConstants.k_is_pigeon_gyro:
            # Pigeon 2 setup
            self.gyro = Pigeon2(Constants.CanIDs.k_pigeon, Constants.DrivetrainConstants.k_canbus_name)
            BaseStatusSignal.set_update_frequency_for_all(
                100,
                self.gyro.get_yaw(),
                self.gyro.get_angular_velocity_y_device()
            )
            self.gyro.optimize_bus_utilization(optimized_freq_hz=20) # Change this to 0 after testing irl

        else:
            self.gyro = AHRS.create_spi(update_rate_hz=100)

        # Odometry
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics, # The wheel locations on the robot
            self.get_yaw(), # The current angle of the robot
            Drivetrain.get_module_positions(),
            starting_pose
        )
        
        # Send Reset Yaw command to Shuffleboard
        Shuffleboard.getTab("Main").add(
            "Reset Yaw",
            self.runOnce(self.reset_yaw) # Simple InstantCommand, nothing crazy
        ).withWidget(BuiltInWidgets.kCommand)

        # Field Widget
        self.field = Field2d()
        Shuffleboard.getTab("Main").add("Field", self.field).withWidget(BuiltInWidgets.kField)
        
        # Configure PathPlanner
        AutoBuilder.configureHolonomic(
            self.get_pose,
            lambda pose: self.reset_pose(pose),
            self.get_robot_speed,
            lambda speeds: self.drive(speeds.vx, speeds.vy, speeds.omega, is_field_relative=False),
            self.path_follower_config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # "Hey Caden, when do we flip the path?"
            self # "Yes, this is the drivetrain. Why would I configure an AutoBuilder for an intake, pathplannerlib?"
        )

        # Logs the target pose
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.target_pose_publisher.set(pose))

        # Shows the active path on the field widget
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("active_path").setPoses(poses))

        ## NetworkTable Publishing (for logging)
        drivetrain_nt = NetworkTableInstance.getDefault().getTable("Drivetrain")

        # Odometry
        self.robot_pose_publisher = drivetrain_nt.getStructTopic("RobotPose", Pose2d).publish()
        self.latency_comp_publisher = drivetrain_nt.getStructTopic("LatencyCompPose", Pose2d).publish()
        self.target_pose_publisher = drivetrain_nt.getStructTopic("PPTarget", Pose2d).publish()
        self.vision_pose_publisher = drivetrain_nt.getStructTopic("VisionPose", Pose2d).publish()

        # Swerve
        self.module_state_publisher = drivetrain_nt.getStructArrayTopic("ModuleStates", SwerveModuleState).publish()
        self.module_target_publisher = drivetrain_nt.getStructArrayTopic("ModuleTargets", SwerveModuleState).publish()
        self.skid_ratio_publisher = drivetrain_nt.getFloatTopic("SkidRatio").publish()
        
        self._last_odometry_update = 0

    def drive(self, velocity_x: float, velocity_y: float, velocity_rot: float, center_of_rotation: Translation2d = Translation2d(), is_field_relative = True) -> None:
        """Converts from robot speeds into module states, then updates all module target states.

        Args:
            velocity_x (float): Forward speed in m/s.
            velocity_y (float): Sideways speed in m/s, positive values strafe to the left.
            velocity_rot (float): Rotation speed in rad/s, CCW+.
            center_of_rotation (Translation2d, optional): Center of rotation for rotating around an object. Defaults to Translation2d(), the center of the robot.
            is_field_relative (bool, optional): Converts speeds into robot-centric speeds. Set to False if passing in robot-centric speeds already. Defaults to True.
        """

        # Create ChassisSpeeds, then discretize
        speeds = ChassisSpeeds.discretize(
            ChassisSpeeds(
                velocity_x,
                velocity_y,
                velocity_rot
            ), 
            0.02
        )

        if is_field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, self.get_yaw())

        module_speeds = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
        self.apply_module_targets(module_speeds)

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def get_interpolated_pose(self) -> Pose2d:
        """Interpolates the current pose of the robot by transforming the estimated position by the velocity."""
        estimated_position = self.get_pose()

        estimated_velocity = self.get_robot_speed()

        latency = Timer.getFPGATimestamp() - self._last_odometry_update

        velocity_transform = Transform2d(estimated_velocity.vx*latency, estimated_velocity.vy*latency, Rotation2d(estimated_velocity.omega*latency))

        return estimated_position.transformBy(velocity_transform)
    
    def reset_pose(self, pose: Pose2d) -> None:
        """Resets the robot's recorded position on the field via odometry."""
        
        self.odometry.resetPosition(
            self.get_yaw(),
            Drivetrain.get_module_positions(),
            pose
        )

    def periodic(self) -> None:

        self.update_odometry()
        self.update_vision_estimates()

        # Update the field pose
        self.field.setRobotPose(self.odometry.getEstimatedPosition())

        # Log everything
        self.skid_ratio_publisher.set(self.get_skidding_ratio())

        ## Show swerve modules on robot
        if not DriverStation.isFMSAttached() and not RobotBase.isReal():
            module_angles = Drivetrain.get_module_angles()

            module_poses = []
            for i in range(len(self.modules)):

                translation = Constants.DrivetrainConstants.k_module_locations[i]

                sim_offset = Translation2d(
                    math.copysign(0.25, translation.X()), 
                    math.copysign(0.25, translation.Y())
                )

                module_poses.append(
                    self.field.getRobotPose().transformBy(
                        Transform2d(
                            translation - sim_offset,
                            module_angles[i]
                        )
                    )
                )
            self.field.getObject("modules").setPoses(module_poses)
        elif len(self.field.getObject("modules").getPoses()) > 1:
            self.field.getObject("modules").setPose(-1000, -1000, Rotation2d())

        # Update Gyro widget
        SmartDashboard.putNumber("Yaw", -self.get_yaw().degrees())

        # Update Skid Ratio
        SmartDashboard.putNumber("Skidding Ratio", Drivetrain.get_skidding_ratio())

    def update_odometry(self) -> None:
        """Reads module positions to update odometry (wow)."""

        timestamp = Timer.getFPGATimestamp()

        self.odometry.updateWithTime(
            timestamp,
            self.get_yaw(), 
            Drivetrain.get_module_positions()
        )

        # Log Everything
        self.module_target_publisher.set(list(Drivetrain.get_module_targets()))
        self.module_state_publisher.set(list(Drivetrain.get_module_states()))

        self.robot_pose_publisher.set(self.odometry.getEstimatedPosition())
        self.latency_comp_publisher.set(self.get_interpolated_pose())

        self._last_odometry_update = timestamp

    def update_vision_estimates(self) -> None:
        """Uses Limelight MegaTag to help prevent pose drift."""

        add_vision_estimate = Constants.LimelightConstants.k_enable_vision_odometry
        if not Constants.LimelightConstants.k_use_mega_tag_2: # Mega Tag 1
            
            mega_tag1 = LimelightHelpers.get_botpose_estimate_wpiblue(Constants.LimelightConstants.k_limelight_name)

            # Check if we're confident on where we are on the field
            if mega_tag1.tag_count == 1 and len(mega_tag1.raw_fiducials) == 1:

                if mega_tag1.raw_fiducials[0].ambiguity > .7 \
                    or mega_tag1.raw_fiducials[0].dist_to_camera > 3: # Don't trust mega tag 1 if we're not close to the april tags

                    add_vision_estimate = False 
            
            elif mega_tag1.tag_count == 0:
                add_vision_estimate = False # Obviously, don't add vision measurements if it doesn't see any apriltags

            # Add Vision Measurement
            if add_vision_estimate:
                self.odometry.setVisionMeasurementStdDevs(Constants.LimelightConstants.k_standard_deviations)
                self.odometry.addVisionMeasurement(
                    mega_tag1.pose,
                    mega_tag1.timestamp_seconds
                )

                self.vision_pose_publisher.set(mega_tag1.pose)
        
        else: # Mega Tag 2

            # Set Robot Orientation
            LimelightHelpers.set_robot_orientation(
                Constants.LimelightConstants.k_limelight_name,
                self.get_yaw().degrees(),
                self.get_yaw_rate(),
                0, 0, 0, 0
            )

            mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(Constants.LimelightConstants.k_limelight_name)

            # If we're spinning or we don't see an apriltag, don't add vision measurements
            if abs(self.get_yaw_rate()) > 720 or mega_tag2.tag_count == 0:
                add_vision_estimate = False

            # Add Vision Measurement
            if add_vision_estimate:
                self.odometry.setVisionMeasurementStdDevs(Constants.LimelightConstants.k_standard_deviations)
                self.odometry.addVisionMeasurement(
                    mega_tag2.pose,
                    mega_tag2.timestamp_seconds
                )

                self.vision_pose_publisher.set(mega_tag2.pose)

    def simulationPeriodic(self):

        if Constants.DrivetrainConstants.k_is_pigeon_gyro:
            gyro_sim = self.gyro.sim_state
            gyro_sim.set_supply_voltage(RobotController.getBatteryVoltage())
            gyro_sim.add_yaw(self.get_robot_speed().omega_dps * 0.02)

    @staticmethod
    def get_skidding_ratio() -> float:
        """
        Translated from https://www.chiefdelphi.com/t/has-anyone-successfully-implemented-orbits-odometry-skid-detection/468257/3
        Built from concepts mentioned in 1690's Software Session, https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674

        Returns the skidding ratio to determine how much the chassis is skidding.
        The skidding ratio is the ratio between the maximum and minimum magnitude of the translational speed of the modules.
        """
        def module_state_to_velocity_vector(module_state: SwerveModuleState) -> Translation2d:
            return Translation2d(module_state.speed, module_state.angle)

        module_states = Drivetrain.get_module_states()

        state_rotation = Drivetrain.kinematics.toSwerveModuleStates(ChassisSpeeds(0, 0, Drivetrain.get_robot_speed().omega))

        module_translation_magnitudes = []

        for i in range(len(module_states)):
            module_state_vector = module_state_to_velocity_vector(module_states[i])

            module_rotation_vector = module_state_to_velocity_vector(state_rotation[i])
            module_translation_vector = module_state_vector - module_rotation_vector

            module_translation_magnitudes.append(module_translation_vector.norm())

        max_translation = 0
        min_translation = math.inf
        for mag in module_translation_magnitudes:
            max_translation = max(max_translation, mag)
            min_translation = min(min_translation, mag)
        
        try:
            return max_translation / min_translation
        except ZeroDivisionError:
            return 1.0

    def get_yaw(self) -> Rotation2d:
        """Gets the rotation of the robot."""
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value) if Constants.DrivetrainConstants.k_is_pigeon_gyro else Rotation2d.fromDegrees(-self.gyro.getYaw())
    
    def get_yaw_rate(self) -> float:
        return self.gyro.get_angular_velocity_y_world().value if Constants.DrivetrainConstants.k_is_pigeon_gyro else -self.gyro.getRate()
    
    def reset_yaw(self) -> None:
        self.gyro.set_yaw(0) if Constants.DrivetrainConstants.k_is_pigeon_gyro else self.gyro.reset()

    @staticmethod
    def get_module_angles() -> tuple[Rotation2d]:
        """Returns the angle for every module as a tuple of Rotation2d."""

        angles = []
        for module in Drivetrain.modules:
            angles.append(module.get_angle())
        return tuple(angles)

    @staticmethod
    def get_module_positions() -> tuple[SwerveModulePosition]:
        """Returns all reported SwerveModulePositions for every module."""

        positions = []
        for module in Drivetrain.modules:
            positions.append(module.get_position())
        return tuple(positions)
    
    @staticmethod
    def get_module_states() -> tuple[SwerveModuleState]:
        """Returns all reported SwerveModuleStates for every module."""

        states = []
        for module in Drivetrain.modules:
            states.append(module.get_state())
        return tuple(states)
    
    @staticmethod
    def get_module_targets() -> tuple[SwerveModuleState]:
        """Returns all module target states."""

        targets = []
        for module in Drivetrain.modules:
            targets.append(module.get_target())
        return targets

    @staticmethod
    def get_robot_speed() -> ChassisSpeeds:
        """Returns the robots current speed (non-field relative)"""
        
        return Drivetrain.kinematics.toChassisSpeeds(Drivetrain.get_module_states())

    @staticmethod
    def apply_module_targets(states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """Sends the given module states to each module."""
        
        # Make sure we aren't traveling at unrealistic speeds
        states = Drivetrain.kinematics.desaturateWheelSpeeds(
            states, Constants.DrivetrainConstants.k_max_drive_speed
        )
        
        # Set each state to the correct module
        for i, module in enumerate(Drivetrain.modules):

            if states[i].speed == 0:
                module.stop()
            else:
                module.set_desired_state(states[i])
