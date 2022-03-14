import sys

from quadsim.src.controllers.pid import PID_Controller
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.tests.constants import rollKp, rollKi, rollKd,\
    pitchKp, pitchKi, pitchKd,\
    yawKp, yawKi, yawKd, T, limRoll, limPitch, limYaw


def test_pid(plot=True, save_plot=False):

    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)

    test_controller(pid, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_pid(True, False)
