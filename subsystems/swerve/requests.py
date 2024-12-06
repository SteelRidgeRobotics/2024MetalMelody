from typing import Self

from phoenix6 import StatusCode, units
from phoenix6.swerve.requests import FieldCentricFacingAngle, ForwardPerspectiveValue, SwerveRequest
from phoenix6.swerve import SwerveModule, SwerveControlParameters
from wpimath import angleModulus
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import *


class ProfiledFieldCentricFacingAngle(SwerveRequest):
    """
    Drives the swerve drivetrain in a field-centric manner, maintaining a
    specified heading angle to ensure the robot is facing the desired direction.
    Rotation to the target direction is profiled using a trapezoid profile.

    When users use this request, they specify the direction the robot should
    travel oriented against the field, and the direction the robot should be facing.

    An example scenario is that the robot is oriented to the east, the VelocityX
    is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180 degrees.
    In this scenario, the robot would drive northward at 5 m/s and turn clockwise
    to a target of 180 degrees.

    This control request is especially useful for autonomous control, where the
    robot should be facing a changing direction throughout the motion.
    """
    
    
    def __init__(self, constraints: TrapezoidProfile.Constraints) -> None:
        """Creates a new profiled ProfiledFieldCentricFacingAngle request with 
        the given constraints.

        :param constraints: Constraints for the trapezoid profile
        :type constraints: TrapezoidProfile.Constraints
        """
        self.velocity_x = 0
        self.velocity_y = 0
        self.target_direction = Rotation2d()
        
        self.deadband = 0
        self.rotatonal_deadband = 0
        self.center_of_rotation = Translation2d()
        
        self.drive_request_type = SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        self.steer_request_type = SwerveModule.SteerRequestType.POSITION
        self.desaturate_wheel_speeds = True
        
        self.forward_perspective = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
        
        self._field_centric_facing_angle = FieldCentricFacingAngle()
        
        self.heading_controller = self._field_centric_facing_angle.heading_controller
        
        self._profile = TrapezoidProfile(constraints)
        self._setpoint = TrapezoidProfile.State()
        self._goal = TrapezoidProfile.State()
    
    def apply(self, parameters: SwerveControlParameters, modules_to_apply: list[SwerveModule]) -> StatusCode:
        self._goal = TrapezoidProfile.State(self.target_direction.radians())
        
        current_angle = parameters.current_pose.rotation()
        if self.forward_perspective is ForwardPerspectiveValue.OPERATOR_PERSPECTIVE:
            # If we're operator perspective, rotate our current heading 
            # back by the angle
            current_angle = current_angle.rotateBy(-parameters.operator_forward_direction)
        
        # From ProfiledPIDController::calculate
        
        # Get error which is the smallest distance between goal and 
        # measurement
        goal_min_distance = angleModulus(self._goal.position - current_angle.radians())
        setpoint_min_distance = angleModulus(self._setpoint.position - current_angle.radians())
        
        # Recompute the profile goal with the smallest error, thus 
        # giving the shortest path. The goal may be outside the input 
        # range after this operation, but that's OK because the 
        # controller will still go there and report an error of zero. 
        # In other words, the setpoint only needs to be offset from the 
        # measurement by the input range modulus; they don't need to be 
        # equal.
        self._goal = TrapezoidProfile.State(goal_min_distance 
                               + current_angle.radians())
        self._setpoint = TrapezoidProfile.State(setpoint_min_distance 
                                   + current_angle.radians())
        
        self._setpoint = self._profile.calculate(parameters.update_period, self._setpoint, self._goal)
        return (self._field_centric_facing_angle
                .with_velocity_x(self.velocity_x)
                .with_velocity_y(self.velocity_y)
                .with_target_direction(Rotation2d(self._setpoint.position))
                .with_target_rate_feedforward(self._setpoint.velocity)
                .with_deadband(self.deadband)
                .with_rotational_deadband(self.rotatonal_deadband)
                .with_center_of_rotation(self.center_of_rotation)
                .with_drive_request_type(self.drive_request_type)
                .with_steer_request_type(self.steer_request_type)
                .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
                .with_forward_perspective(self.forward_perspective)
                .apply(parameters, modules_to_apply)
        )
        
    def reset_profile(self, current_heading: Rotation2d) -> None:
        """Resets the profile used for the target direction.

        :param current_heading: The current heading of the robot
        :type current_heading: Rotation2d
        """
        self._setpoint = TrapezoidProfile.State(current_heading.radians())
        
    def with_velocity_x(self, new_velocity_x: meters_per_second) -> 'ProfiledFieldCentricFacingAngle':
        """Modifies the velocity_x parameter and returns itself.
        
        The velocity in the X direction, in m/s. X is defined as forward
        according to WPILib convention, so this determines how fast to 
        travel forward.

        :param new_velocity_x: Parameter to modify
        :type new_velocity_x: float
        :return: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """
        
        self.velocity_x = new_velocity_x
        return self
    
    def with_velocity_y(self, new_velocity_y: meters_per_second) -> 'ProfiledFieldCentricFacingAngle':
        """Modifies the velocity_x parameter and returns itself.
        
        The velocity in the Y direction, in m/s. Y is defined as to the 
        left according to WPILib convention, so this determines how 
        fast to travel to the left.

        :param new_velocity_y: Parameter to modify
        :type new_velocity_y: float
        :return: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """
        
        self.velocity_y = new_velocity_y
        return self
    
    def with_target_direction(self, new_target_direction: Rotation2d) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the target_direction parameter and returns itself.

        The desired direction to face.
        0 Degrees is defined as in the direction of the X axis.
        As a result, a TargetDirection of 90 degrees will point along
        the Y axis, or to the left.

        :param new_target_direction: Parameter to modify
        :type new_target_direction: Rotation2d
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.target_direction = new_target_direction
        return self
    
    def with_deadband(self, new_deadband: meters_per_second) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the deadband parameter and returns itself.

        The allowable deadband of the request, in m/s.

        :param new_deadband: Parameter to modify
        :type new_deadband: meters_per_second
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.deadband = new_deadband
        return self
    
    def with_rotational_deadband(self, new_rotational_deadband: units.radians_per_second) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the rotational_deadband parameter and returns itself.

        The rotational deadband of the request, in radians per second.

        :param new_rotational_deadband: Parameter to modify
        :type new_rotational_deadband: radians_per_second
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.rotational_deadband = new_rotational_deadband
        return self
    
    def with_center_of_rotation(self, new_center_of_rotation: Translation2d) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the center_of_rotation parameter and returns itself.

        The center of rotation the robot should rotate around. This is (0,0) by default,
        which will rotate around the center of the robot.

        :param new_center_of_rotation: Parameter to modify
        :type new_center_of_rotation: Translation2d
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.center_of_rotation = new_center_of_rotation
        return self
    
    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the drive_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_drive_request_type: Parameter to modify
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.drive_request_type = new_drive_request_type
        return self
    
    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the steer_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_steer_request_type: Parameter to modify
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.steer_request_type = new_steer_request_type
        return self
    
    def with_desaturate_wheel_speeds(self, new_desaturate_wheel_speeds: bool) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the desaturate_wheel_speeds parameter and returns itself.

        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.

        :param new_desaturate_wheel_speeds: Parameter to modify
        :type new_desaturate_wheel_speeds: bool
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.desaturate_wheel_speeds = new_desaturate_wheel_speeds
        return self

    def with_forward_perspective(self, new_forward_perspective: ForwardPerspectiveValue) -> 'ProfiledFieldCentricFacingAngle':
        """
        Modifies the forward_perspective parameter and returns itself.

        The perspective to use when determining which direction is forward.

        :param new_forward_perspective: Parameter to modify
        :type new_forward_perspective: ForwardPerspectiveValue
        :returns: this object
        :rtype: ProfiledFieldCentricFacingAngle
        """

        self.forward_perspective = new_forward_perspective
        return self
    