import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem

class IntakeAndPivot(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.intakeMotor = phoenix6.hardware.TalonFX(MotorIDs.INTAKEMOTOR)
        self.pivotMotor = phoenix6.hardware.TalonFX(MotorIDs.PIVOTMOTOR)
        self.choking = False

        CommandScheduler.getInstance().registerSubsystem(self)

    #####[[ INTAKE FUNCTIONS ]]#####

    def spit(self) -> None: # drop note
        self.intakeMotor.set_control(phoenix6.controls.DutyCycleOut(1.0))

    def swallow(self) -> None: # intake note
        self.intakeMotor.set_control(phoenix6.controls.DutyCycleOut(-1.0))

    def hasNote(self) -> bool: # use beam break to see if note is inside intake
        pass

    def periodic(self) -> None: # update whether the robot has the note or not
        if self.hasNote():
            self.choking = True
        else:
            self.choking = False

    #####[[ PIVOT FUNCTIONS ]]#####
            
    def pivot(self) -> None:
        pass

    





