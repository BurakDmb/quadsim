import sys

from quadsim.src.controllers.linear_mpc import Linear_MPC
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import n_horizon, \
    control_freq, simulation_freq, t_end


from quadsim.src.envs.quad import DeterministicQuad
from quadsim.src.envs.quad import linear_quad_dynamics
import time


def test_LinearMPC(plot=False, save_plot=False, loadmodel=False):
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    start = time.time()
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    linear_mpc = Linear_MPC(t_end=t_end,
                            n_horizon=n_horizon,
                            c_step=1/control_freq,
                            s_step=1/simulation_freq,
                            env=env)
    test_controller(linear_mpc, t_end, plot=plot, save_plot=save_plot)
    end = time.time()
    print(end-start)


if __name__ == "__main__":
    test_LinearMPC(True, False)
