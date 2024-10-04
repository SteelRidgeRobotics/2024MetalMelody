from commands2 import Command
from subsystems.swivel import Swivel
from subsystems.indexer import Indexer
from subsystems.launcher import Launcher
from constants import *

class RevLauncher(Command):
    
    def __init__(self, swivel: Swivel, indexer: Indexer, launcher: Launcher, angle):
        super().__init__()
        
        self.swivel = swivel
        self.indexer = indexer
        self.launcher = launcher
        self.addRequirements(self.swivel, self.indexer)

    def initialize(self):
        self.launcher.rev()

    def isFinished(self) -> bool:
        return self.launcher.get_velocity() > LauncherConstants.SHOOTPERCENT