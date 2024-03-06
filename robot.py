from commands.control_swerve_speed import ControlSwerveSpeed
from commands.reset_pivot import ResetPivot
import commands2
from commands2.timedcommandrobot import seconds
from phoenix6.signal_logger import SignalLogger
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from wpilib import DriverStation, RobotBase
from wpilib.cameraserver import CameraServer

class MetalMelody(commands2.TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):
        DriverStation.silenceJoystickConnectionWarning(True)
        
        SignalLogger.enable_auto_logging(False)
        
        self.container = RobotContainer()
        
        if RobotBase.isReal():
            CameraServer.launch()
            
    def robotPeriodic(self) -> None:
        pass
    
    def _simulationPeriodic(self) -> None:
        pass
        
    def disabledInit(self) -> None:
        SignalLogger.stop()
        commands2.CommandScheduler.getInstance().cancelAll()
        
    def disabledPeriodic(self) -> None:
        pass
        
    def autonomousInit(self) -> None:
        self.container.runSelectedAutoCommand()

    def teleopInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(ControlSwerveSpeed(self.container.elevator, self.container.swerve, self.container.driverController.getLeftBumper))
    
    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().schedule(ResetPivot(self.container.intake))
    
    def testExit(self) -> None:
        SignalLogger.stop()
