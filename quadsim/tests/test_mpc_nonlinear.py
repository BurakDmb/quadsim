import sys
sys.path.insert(0, './')

from quadsim.src.controllers.nonlinear_mpc import Nonlinear_MPC  # noqa: E402
from quadsim.scripts.utils import test_controller  # noqa: E402
from quadsim.tests.constants import n_horizon, \
    control_freq, simulation_freq, t_end  # noqa: E402

from quadsim.src.envs.quad import DeterministicQuad  # noqa: E402
from quadsim.src.envs.quad import linear_quad_dynamics  # noqa: E402
import time  # noqa: E402


def test_NonlinearMPC(plot=False, save_plot=False, loadmodel=False):
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
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
