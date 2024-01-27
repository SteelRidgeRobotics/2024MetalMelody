import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.elevatorMotor = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR)

        self.elevator_config = phoenix6.configs.TalonFXConfiguration()

        self.elevatorMotor.configurator.apply(self.elevator_config)

        CommandScheduler.getInstance().registerSubsystem(self)


    #####[[ ELEVATOR FUNCTIONS ]]#####
            
             
    def toggle(self) -> None: # Move elevator down
        
        self.sensorPos = self.elevatorMotor.get_rotor_position().value()

        if self.sensorPos == MotorConstants.TOPPOSITION:
            
            self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(MotorConstants.BOTTOMPOSITION))
        
        
        elif self.sensorPos == MotorConstants.BOTTOMPOSITION:
            
            self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(MotorConstants.TOPPOSITION))