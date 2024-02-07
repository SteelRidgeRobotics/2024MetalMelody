import phoenix6
from constants import * 
from commands2 import CommandScheduler, Subsystem
from wpimath.filter import SlewRateLimiter

class IntakeAndPivot(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.intakeMotor = phoenix6.hardware.TalonFX(MotorIDs.INTAKEMOTOR)
        self.pivotMotor = phoenix6.hardware.TalonFX(MotorIDs.PIVOTMOTOR)

        self.slew = SlewRateLimiter(.5)

        self.holding = False

        self.desired_speed = 0
        self.pivotIndex = 0 #index for the pivot angles list in constants
        self.desired_angle = 90

        CommandScheduler.getInstance().registerSubsystem(self)


    #####[[ INTAKE FUNCTIONS ]]#####


    def disencumber(self) -> None: # drop note
        self.desired_speed = -IntakeConstants.INTAKESPEED*1.5


    def consume(self) -> None: # intake note
        self.desired_speed = IntakeConstants.INTAKESPEED


    def hold(self) -> None: # hold note
        self.desired_speed = 0


    def hasNote(self) -> bool: # use beam break to see if note is inside intake
        pass


    def periodic(self) -> None: # update whether the robot has the note or not

        if self.hasNote():
            self.holding = True
        else:
            self.holding = False
            
        self.intakeMotor.set_control(phoenix6.controls.DutyCycleOut(self.slew.calculate(self.desired_speed)))
        self.pivotMotor.set_control(phoenix6.controls.MotionMagicDutyCycle(self.slew.calculate(self.desired_angle)))


    #####[[ PIVOT FUNCTIONS ]]#####
            
    def pivotDown(self) -> None: #set motor position to something specific

        self.pivotIndex -= 1   #(self.pivotIndex + 1) % len(IntakeConstants.PIVOTPOS)
        if self.pivotIndex < 1:
            self.pivotIndex = 1

    def pivotUp(self) -> None:
        
        self.pivotIndex += 1
        if self.pivotIndex > 3:
            self.pivotIndex = 3