import sys
sys.path.insert(0, './')

from quadsim.src.controllers.lqg import LQG  # noqa: E402
from quadsim.scripts.utils import test_controller  # noqa: E402
from quadsim.tests.constants import t_end  # noqa: E402

from quadsim.src.envs.quad import DeterministicQuad  # noqa: E402
from quadsim.src.envs.quad import linear_quad_dynamics  # noqa: E402


def test_lqg(plot=False, save_plot=False, loadmodel=False):
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    lqr = LQG(env)
    test_controller(lqr, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_lqg(True, False)
