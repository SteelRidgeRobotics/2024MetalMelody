from commands2 import Subsystem
import phoenix6
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from constants import *
from commands.mode_toggle import ModeToggle as mode_toggle

class Launcher(Subsystem):

    def __init__(self) -> None:
        super().__init__()

        self.upperLeftLauncherMotor = TalonFX(MotorIDs.LEFT_UPPER_LAUNCHER)
        self.lowerLeftLauncherMotor = TalonFX(MotorIDs.LEFT_LOWER_LAUNCHER)
        self.upperRightLauncherMotor = TalonFX(MotorIDs.RIGHT_UPPER_LAUNCHER)
        self.lowerRightLauncherMotor = TalonFX(MotorIDs.RIGHT_LOWER_LAUNCHER)

        self.followRequest = phoenix6.controls.Follower(MotorIDs.LEFT_UPPER_LAUNCHER, False)
        self.lowerRightLauncherMotor.set_control(self.followRequest)
        self.lowerLeftLauncherMotor.set_control(self.followRequest.with_master_id(MotorIDs.RIGHT_UPPER_LAUNCHER))

        self.invertConfig = phoenix6.configs.MotorOutputConfigs()

        self.invertConfig.inverted = phoenix6.configs.talon_fx_configs.InvertedValue.CLOCKWISE_POSITIVE

        self.upperLeftLauncherMotor.configurator.apply(self.invertConfig)

        #self.upperLeftLauncherMotor.set_control(DutyCycleOut(0.1))
        #self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0.1))
        

    def launch(self) -> None:
        self.upperLeftLauncherMotor.set_control(DutyCycleOut(0.3))
        self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0.3))

    def stop(self) -> None:

        self.upperLeftLauncherMotor.set_control(DutyCycleOut(0))
        self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0))


    def periodic(self) -> None:

        if mode_toggle.get_mode() == Modes.LAUNCHER:
            self.upperLeftLauncherMotor.set_control(DutyCycleOut(0.1))
            self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0.1))
        else:
            self.stop()

        SmartDashboard.putNumber("Mode ", mode_toggle.get_mode())
        
