from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from subsystems.swerve import SwerveSubsystem

class RobotState:
    def __init__(self, swerve: SwerveSubsystem):
        self.swerve = swerve

        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("currentPose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassisSpeeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("Odometry Frequency").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("moduleStates", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("moduleTargets", SwerveModuleState).publish()

    def log_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Logs desired info with the given swerve state. Called by the 
        Phoenix 6 drivetrain method every time the odometry thread is 
        updated.
        """

        self._field.setRobotPose(state.pose)
        self._current_pose.set(state.pose)

        self._odom_freq.set(1.0 / state.odometry_period)

        self._module_states.set(state.module_states)
        self._module_targets.set(state.module_targets)
        self._chassis_speeds.set(state.speeds)

    def get_current_pose(self) -> Pose2d:
        
        """Returns the current pose of the robot on the field (blue-side origin)."""
        return self.swerve.get_state().pose
    
    def get_latency_compensated_pose(self, dt: float) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin),
        compensated for latency.

        :param dt: The amount of time in seconds since the last 
            update.
        :type dt: float
        :return: The current pose of the robot on the field with 
            latency compensation.
        :rtype: Pose2d
        """
        state = self.swerve.get_state()
        speeds = state.speeds
        pose = state.pose

        return Pose2d(pose.X() + speeds.vx * dt,
                      pose.Y() + speeds.vy * dt,
                      pose.rotation() + speeds.omega * dt)
    