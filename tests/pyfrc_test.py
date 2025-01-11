import pytest
import typing

if typing.TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController

@pytest.mark.filterwarnings("ignore")
def test_impatient(control: "TestController"):

    with control.run_robot():
        # Run disabled for a second
        control.step_timing(seconds=1, autonomous=True, enabled=False)

        # Run autonomous + enabled for a second
        control.step_timing(seconds=1, autonomous=True, enabled=True)

        # You get the idea
        control.step_timing(seconds=1, autonomous=False, enabled=False)

        # Teleop
        control.step_timing(seconds=1, autonomous=False, enabled=True)
