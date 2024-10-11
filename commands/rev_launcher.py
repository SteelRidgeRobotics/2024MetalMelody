from commands2 import Command
from subsystems.swivel import Swivel
from subsystems.launcher import Launcher
from constants import *

class RevLauncher(Command):
    
    def __init__(self, launcher: Launcher, angle):
        super().__init__()
        
        self.launcher = launcher
        self.addRequirements(self.launcher)

    def initialize(self):
        self.launcher.rev()

    def execute(self):
        

        return super().execute()

    def isFinished(self) -> bool:
        return self.launcher.get_velocity() > LauncherConstants.SHOOTPERCENT