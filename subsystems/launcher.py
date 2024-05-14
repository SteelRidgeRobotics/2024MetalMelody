from commands2 import Subsystem
import phoenix6
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut
from constants import MotorIDs

class Launcher(Subsystem):

    def __init__(self) -> None:
        super().__init__()

        self.upperLeftLauncherMotor = TalonFX(MotorIDs.LEFT_UPPER_LAUNCHER)
        self.lowerLeftLauncherMotor = TalonFX(MotorIDs.LEFT_LOWER_LAUNCHER)
        self.upperRightLauncherMotor = TalonFX(MotorIDs.RIGHT_UPPER_LAUNCHER)
        self.lowerRightLauncherMotor = TalonFX(MotorIDs.RIGHT_LOWER_LAUNCHER)

        self.followRequest = phoenix6.controls.Follower(MotorIDs.LEFT_UPPER_LAUNCHER)
        self.upperRightLauncherMotor.set_control(self.followRequest)
        self.lowerRightLauncherMotor.set_control(self.followRequest.with_master_id(MotorIDs.LEFT_LOWER_LAUNCHER))

        self.invertConfig = phoenix6.configs.MotorOutputConfigs()

        self.invertConfig.inverted = phoenix6.configs.talon_fx_configs.InvertedValue.CLOCKWISE_POSITIVE

        self.lowerLeftLauncherMotor.configurator.apply(self.invertConfig)

        self.upperLeftLauncherMotor.set_control(DutyCycleOut(0.2))
        self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0.2))
        

    def launch(self) -> None:
        self.upperLeftLauncherMotor.set_control(DutyCycleOut(1))
        self.lowerLeftLauncherMotor.set_control(DutyCycleOut(1))

    def prime(self) -> None:
        self.upperLeftLauncherMotor.set_control(DutyCycleOut(0.2))
        self.lowerLeftLauncherMotor.set_control(DutyCycleOut(0.2))
        
    