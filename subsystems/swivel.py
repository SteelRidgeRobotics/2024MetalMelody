from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from constants import MotorIDs

class Swivel(Subsystem):

    motor1 = TalonFX(MotorIDs.SWIVELMOTOR_1)
    motor2 = TalonFX(MotorIDs.SWIVELMOTOR_2)


    def sendToShooter(self) -> None:
        pass
        
    