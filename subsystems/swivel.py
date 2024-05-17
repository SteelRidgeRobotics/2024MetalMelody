from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from constants import MotorIDs

class Swivel(Subsystem):
    
    def __init__(self) -> None:
        super().__init__()
        
        self.motor1 = TalonFX(MotorIDs.SWIVELMOTOR_1)
        self.motor2 = TalonFX(MotorIDs.SWIVELMOTOR_2)



    def sendToShooter(self) -> None:
        pass
        
    