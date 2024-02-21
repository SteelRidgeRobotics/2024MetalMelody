from commands2 import Command
from wpilib import Timer
from wpilib.interfaces import GenericHID

class VibrateController(Command):
    
    def __init__(self, controller: GenericHID, rumble_type: GenericHID.RumbleType, duration: int):
        super().__init__()
        
        self.controller = controller
        self.rumble_type = rumble_type
        self.duration = duration
        self.timer = Timer()
    
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.controller.setRumble(self.rumble_type, 1)
        
    def isFinished(self) -> bool:
        return self.timer.get() >= self.duration
    
    def end(self, interrupted: bool):
        self.controller.setRumble(self.rumble_type, 0)
        self.timer.stop()