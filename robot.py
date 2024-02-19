import commands2
from commands2.timedcommandrobot import seconds
from phoenix6.signal_logger import SignalLogger
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from wpilib.cameraserver import CameraServer

class DumpsterFire(commands2.TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):
        SignalLogger.set_path("/home/lvuser/logs")
        SignalLogger.enable_auto_logging(True)
        CameraServer.launch('vision.py')
        self.container = RobotContainer()
        SignalLogger.start()

    def robotPeriodic(self):
        self.container.updateOdometry()
        commands2.CommandScheduler.getInstance().run()
        
    def disabledInit(self) -> None:
        pass
        #SignalLogger.stop()
        
    def autonomousInit(self) -> None:
        self.container.runSelectedAutoCommand()
        
    def testInit(self) -> None:
        pass
    
    def testExit(self) -> None:
        SignalLogger.stop()
        
    
