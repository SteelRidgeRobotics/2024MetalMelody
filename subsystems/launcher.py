from commands2 import Command, Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut, Follower
from constants import MotorIDs, LauncherConstants
import phoenix6
import wpilib

class Launcher(Subsystem):

    def __init__(self):
        super().__init__()
        self.top_left = TalonFX(MotorIDs.k_top_left_launcher)
        self.bottom_left = TalonFX(MotorIDs.k_bottom_left_launcher)
        self.top_right = TalonFX(MotorIDs.k_top_right_launcher)
        self.bottom_right = TalonFX(MotorIDs.k_bottom_right_launcher)

        self.motor_configuration = phoenix6.configs.TalonFXConfiguration()
        self.motor_configuration.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST

        self.top_left.configurator.apply(self.motor_configuration)
        self.bottom_left.configurator.apply(self.motor_configuration)
        self.top_right.configurator.apply(self.motor_configuration)
        self.bottom_right.configurator.apply(self.motor_configuration)
        
        self.bottom_left.set_control(
            Follower(MotorIDs.k_top_left_launcher, True)
        )

        self.top_right.set_control(
            Follower(MotorIDs.k_top_left_launcher, True)
        )
            
        self.bottom_right.set_control(
            Follower(MotorIDs.k_bottom_right_launcher, False)
        )

        wpilib.SmartDashboard.putNumber("Rev", 0)

    def get_velocity(self):
        return self.top_left.get_rotor_velocity().value

    def rev(self):
        self.top_left.set_control(DutyCycleOut(LauncherConstants.SHOOTPERCENT))
        wpilib.SmartDashboard.putNumber("Rev", self.top_left.get_duty_cycle().value)
        
    def stop(self):
        self.top_left.set_control(DutyCycleOut(LauncherConstants.CONSTANTPERCENT))
        wpilib.SmartDashboard.putNumber("Rev", self.top_left.get_duty_cycle().value)

    
