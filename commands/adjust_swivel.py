from commands2 import Command
from subsystems.swivel import Swivel
from constants import *

class AdjustSwivel(Command):
    
    def __init__(self, swivel: Swivel, angle):
        super().__init__()
        
        self.swivel = swivel
        self.angle = angle
        
        self.addRequirements(self.launcher)

    def execute(self):
        self.swivel.reposition(self.angle)

    def isFinished(self) -> bool:
        return False