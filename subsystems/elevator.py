import phoenix6
from phoenix6.controls import MotionMagicVoltage
from constants import * 
from commands2 import Subsystem
import wpilib

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.elevatorMotorRight = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR_RIGHT)
        self.elevatorMotorLeft = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR_LEFT)
        elevator_config = phoenix6.configs.TalonFXConfiguration()
        elevator_config.slot0.with_k_p(ElevatorConstants.kP)
        elevator_config.motion_magic.with_motion_magic_acceleration(1).with_motion_magic_cruise_velocity(ElevatorConstants.MOTIONMAGICVELOCITY).with_motion_magic_jerk(ElevatorConstants.MOTIONMAGICJERK)
        elevator_config.current_limits.supply_current_limit = ElevatorConstants.CURRENTSUPPLYLIMIT
        elevator_config.current_limits.supply_current_limit_enable = ElevatorConstants.CURRENTSUPPLYLIMIT
        
        self.elevatorMotorRight.configurator.apply(elevator_config)
        self.elevatorMotorLeft.configurator.apply(elevator_config)
        
        self.elevatorMotorLeft.set_position(0)
        self.elevatorMotorRight.set_position(0)

        self.stage = 0

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Stage", self.stage)
        
    def up(self) -> None:
        self.setTo(ElevatorConstants.TOPPOSITION)
        self.stage = 0
        
    def middle(self) -> None:
        self.setTo(ElevatorConstants.MIDDLEPOSITION)
        self.stage = 1
        
    def below(self) -> None:
        self.setTo(ElevatorConstants.BOTTOMPOSITION)
        self.stage = 2
        
    def setTo(self, pos: float) -> None:
        self.elevatorMotorRight.set_control(MotionMagicVoltage(pos))
        self.elevatorMotorLeft.set_control(MotionMagicVoltage(-pos))

    def togglePosition(self) -> None:
        self.setStage((self.stage + 1) % 3)
        
    def setStage(self, stage: int) -> None:
        self.stage = stage
    