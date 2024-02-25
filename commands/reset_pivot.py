from commands2 import Command
from phoenix6.controls import DutyCycleOut
from subsystems.intake import Intake

class ResetPivot(Command):
    
    def __init__(self, intake: Intake):
        super().__init__()
        
        self.intake = intake
        self.addRequirements(self.intake)
        
    def initialize(self):
        self.intake.pivotMotor.set_control(DutyCycleOut(-0.075))
        
    def isFinished(self) -> bool:
        return abs(self.intake.pivotMotor.get_rotor_velocity().value) <= 0.001
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.intake.pivotMotor.set_position(0)
        self.intake.pivotMotor.set_control(DutyCycleOut(0))
        