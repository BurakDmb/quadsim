import sys

from quadsim.src.controllers.lqr import LQR
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import Quad


def test_lqr(plot=False, save_plot=False, loadmodel=False):
    env_config = {
        'is_linear': True,
        'is_stochastic': False,
        't_end': t_end,
        'simulation_freq': 200,
        'control_freq': 200,
        'keep_history': False
    }
    env = Quad(env_config)
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    lqr = LQR(env)
    test_controller(lqr, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_lqr(True, False)
