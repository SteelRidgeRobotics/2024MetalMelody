from commands.reset_pivot import ResetPivot
import commands2
from commands2.timedcommandrobot import seconds
from phoenix6.signal_logger import SignalLogger
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from wpilib import DriverStation
from wpilib.cameraserver import CameraServer

class MetalMelody(commands2.TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):
        DriverStation.silenceJoystickConnectionWarning(True)
        
        SignalLogger.set_path("/home/lvuser/logs")
        SignalLogger.enable_auto_logging(False)
        
        self.container = RobotContainer()
        
        CameraServer.launch()
        
    def disabledInit(self) -> None:
        SignalLogger.stop()
        
    def autonomousInit(self) -> None:
        #self.container.camera.setPipeline(0)
        self.container.runSelectedAutoCommand()

    def teleopInit(self) -> None:
        pass
        #self.container.camera.setPipeline(1)

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(ResetPivot(self.container.intake))
    
    def testExit(self) -> None:
        SignalLogger.stop()
        
    
