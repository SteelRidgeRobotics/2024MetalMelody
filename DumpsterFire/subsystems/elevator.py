import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.elevatorMotor = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR)

        self.elevator_config = phoenix6.configs.TalonFXConfiguration()

        self.isDown = True

        self.elevatorMotor.configurator.apply(self.elevator_config)

        CommandScheduler.getInstance().registerSubsystem(self)


    #####[[ ELEVATOR FUNCTIONS ]]#####
            
             
    def move(self, desiredPos):

        self.elevatorMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(desiredPos))


    
    