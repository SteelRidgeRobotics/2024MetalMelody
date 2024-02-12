import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem
import wpilib

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.elevatorMotor = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR)

        elevator_config = phoenix6.configs.TalonFXConfiguration()
        elevator_config.slot0.with_k_p(ElevatorConstants.kP)
        elevator_config.motion_magic.with_motion_magic_acceleration(ElevatorConstants.MOTIONMAGICACCELERATION).with_motion_magic_cruise_velocity(ElevatorConstants.MOTIONMAGICVELOCITY).with_motion_magic_jerk(ElevatorConstants.MOTIONMAGICJERK)
        elevator_config.current_limits.supply_current_limit = ElevatorConstants.CURRENTSUPPLYLIMIT
        elevator_config.current_limits.supply_current_limit_enable = ElevatorConstants.CURRENTSUPPLYLIMIT
        self.stage = 0
        self.isDown = True

        self.elevatorMotor.configurator.apply(elevator_config)

    #####[[ ELEVATOR FUNCTIONS ]]#####

    def periodic(self) -> None:

        wpilib.SmartDashboard.putNumber("Stage", self.stage)

        if self.stage == 0:
            self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(ElevatorConstants.TOPPOSITION))

        elif self.stage == 1:
           
            self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(ElevatorConstants.MIDDLEPOSITION))

        else:
            self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(ElevatorConstants.BOTTOMPOSITION))

    def togglePosition(self) -> None:

        self.stage = (self.stage + 1) % 3    
    