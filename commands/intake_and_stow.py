from commands2 import Command
from subsystems.intake import Intake
from wpilib import Timer
from wpilib import XboxController

class IntakeAndStow(Command):
    
    def __init__(self, intake: Intake, functions: XboxController):
        super().__init__()
        
        self.intake = intake
        self.functions = functions
        
        self.timer = Timer()
        self.addRequirements(self.intake)
        
        self.already_has_note = self.intake.hasNote()
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.intake.consume()
        
        if self.already_has_note:
            self.cancel()
        
    def execute(self):
        if self.functions.getRightBumper():
            self.cancel()
        
    def isFinished(self) -> bool:
        return self.intake.hasNote() or self.timer.get() >= 10
    
    def end(self, interrupted: bool):
        self.timer.stop()
        if not interrupted:
            self.intake.pivotStow()