from commands2 import Command
from subsystems.intake import Intake
from wpilib import Timer
from wpilib.interfaces import GenericHID

class IntakeAndStow(Command):
    
    def __init__(self, intake: Intake, driver: GenericHID, functions: GenericHID):
        super().__init__()
        
        self.intake = intake
        self.driver = driver
        self.functions = functions
        
        self.timer = Timer()
        
        self.addRequirements(self.intake)
        
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.intake.consume()
        
    def isFinished(self) -> bool:
        return self.intake.hasNote() or self.timer.get() >= 10
    
    def end(self, interrupted: bool):
        self.timer.stop()
        if not interrupted:
            self.intake.pivotStow()