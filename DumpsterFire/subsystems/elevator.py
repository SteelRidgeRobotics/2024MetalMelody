import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.elevatorMotor = phoenix6.hardware.TalonFX(MotorIDs.ELEVATORMOTOR)

        CommandScheduler.getInstance().registerSubsystem(self)


    #####[[ ELEVATOR FUNCTIONS ]]#####
            
             
    def Downwards(self) -> None: # Move elevator down
        self.elevatorMotor.set_control(phoenix6.controls.DutyCycleOut(MotorConstants.ELEVATORSPEED))


    def Upwards(self) -> None: # Elevator goes up
        self.elevatorMotor.set_control(phoenix6.controls.DutyCycleOut(-MotorConstants.ELEVATORSPEED)) 


    



