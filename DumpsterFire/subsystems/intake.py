import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem

class IntakeAndPivot(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.intakeMotor = phoenix6.hardware.TalonFX(MotorIDs.INTAKEMOTOR)
        self.pivotMotor = phoenix6.hardware.TalonFX(MotorIDs.PIVOTMOTOR)

        CommandScheduler.getInstance().registerSubsystem(self)

    def spit(self) -> None:
        self.intakeMotor.set_control(phoenix6.controls.DutyCycleOut(1.0))

    def periodic(self) -> None:
        pass
        
        # Check if note is swallowed via beam break
        # if yes, self.choking = True
        # else, self.choking = False

    def swallow(self) -> None:
        self.intakeMotor.set_control(phoenix6.controls.DutyCycleOut(-1.0))

    def hasNote(self) -> bool:
        pass

    





