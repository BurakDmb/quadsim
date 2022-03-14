import sys


from quadsim.src.controllers.lqg import LQG
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import DeterministicQuad
from quadsim.src.envs.quad import linear_quad_dynamics


def test_lqg(plot=False, save_plot=False, loadmodel=False):
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    lqr = LQG(env)
    test_controller(lqr, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_lqg(True, False)
