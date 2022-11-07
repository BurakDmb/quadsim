import sys

from quadsim.src.controllers.nonlinear_mpc import Nonlinear_MPC
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import n_horizon, \
    control_freq, simulation_freq, t_end

from quadsim.src.envs.quad import Quad
import time


def test_NonlinearMPC(plot=False, save_plot=False, loadmodel=False):
    env = Quad(
        is_linear=True, is_stochastic=False,
        t_end=t_end,
        simulation_freq=250,
        control_freq=250,
        keep_history=False)

    start = time.time()
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    nonlinear_mpc = Nonlinear_MPC(t_end=t_end,
                                  n_horizon=n_horizon,
                                  c_step=1/control_freq,
                                  s_step=1/simulation_freq,
                                  env=env)
    test_controller(nonlinear_mpc, t_end, plot=plot, save_plot=save_plot)
    end = time.time()
    print(end-start)


if __name__ == "__main__":
    test_NonlinearMPC(True, False)
