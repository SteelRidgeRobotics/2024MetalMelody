from commands2 import Command, Subsystem
from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut, Follower
from constants import MotorIDs

class Launcher(Subsystem):

    top_left = TalonFX(MotorIDs.k_top_left_launcher)
    bottom_left = TalonFX(MotorIDs.k_bottom_left_launcher)
    top_right = TalonFX(MotorIDs.k_top_right_launcher)
    bottom_right = TalonFX(MotorIDs.k_bottom_right_launcher)

    bottom_left.set_control(
        Follower(MotorIDs.k_top_left_launcher, True)
    )

    top_right.set_control(
        Follower(MotorIDs.k_top_left_launcher, True)
    )
        
    bottom_right.set_control(
        Follower(MotorIDs.k_bottom_right_launcher, False)
    )    

    def launch(self) -> Command:
        return self.runOnce(
            lambda: self.top_left.set_control(
                DutyCycleOut(0.3)
            )
        )

    def stop(self) -> Command:
        return self.runOnce(
            lambda: self.top_left.set_control(
                DutyCycleOut(0)
            )
        )
