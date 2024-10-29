from commands2 import Command
from subsystems.swivel import Swivel
from subsystems.launcher import Launcher
from constants import *

class RevLauncher(Command):
    
    def __init__(self, launcher: Launcher):
        super().__init__()
        
        self.launcher = launcher
        self.addRequirements(self.launcher)

    def execute(self):
        self.launcher.rev()

        return super().execute()

    def isFinished(self) -> bool:
        return self.launcher.get_velocity() > LauncherConstants.SHOOTPERCENT