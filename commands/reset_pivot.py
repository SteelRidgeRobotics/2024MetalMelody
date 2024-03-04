from commands2 import Command
from phoenix6.controls import DutyCycleOut
from subsystems.intake import Intake
from wpilib import Timer

class ResetPivot(Command):
    
    def __init__(self, intake: Intake):
        super().__init__()
        
        self.intake = intake
        self.addRequirements(self.intake)

        self.timer = Timer()
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.intake.pivotMotor.set_control(DutyCycleOut(-0.2))
        
    def isFinished(self) -> bool:
        return abs(self.intake.pivotMotor.get_rotor_velocity().value) <= 0 and self.timer.get() >= 0.15
    
    def end(self, interrupted: bool):
        self.timer.stop()
        if not interrupted:
            self.intake.pivotMotor.set_position(-0.026)
        self.intake.pivotMotor.set_control(DutyCycleOut(0))
        