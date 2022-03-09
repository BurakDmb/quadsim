# import os
# import numpy as np
import sys
sys.path.insert(0, './')

# from src.controllers.pid import PID_Controller  # noqa: E402

# from src.controllers.lqr import LQR  # noqa: E402
# from src.controllers.lqg import LQG  # noqa: E402

# from src.controllers.linear_mpc import Linear_MPC  # noqa: E402
from src.controllers.nonlinear_mpc import Nonlinear_MPC  # noqa: E402

# from stable_baselines3 import PPO  # noqa: E402
# from stable_baselines3 import SAC  # noqa: E402
# from stable_baselines3 import A2C  # noqa: E402
# from stable_baselines3 import TD3  # noqa: E402
# from stable_baselines3 import DDPG  # noqa: E402


from src.envs.quad import DeterministicQuad  # noqa: E402
# from src.envs.quad import StochasticQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402

from tests.constants import n_horizon, \
    control_freq, simulation_freq  # noqa: E402

from scripts.utils import compare_controller_parameters  # noqa: E402


def compare_parameters(plot=False, save_plot=False, loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    t_end = 5
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    # Controller 1
    nonlinear_mpc1 = Nonlinear_MPC(t_end=t_end,
                                   n_horizon=int(n_horizon/2),
                                   c_step=1/control_freq,
                                   s_step=1/simulation_freq,
                                   env=env)

    # Controller 2
    nonlinear_mpc2 = Nonlinear_MPC(t_end=t_end,
                                   n_horizon=n_horizon,
                                   c_step=1/control_freq,
                                   s_step=1/simulation_freq,
                                   env=env)
    # Controller 3
    nonlinear_mpc3 = Nonlinear_MPC(t_end=t_end,
                                   n_horizon=int(n_horizon*5),
                                   c_step=1/control_freq,
                                   s_step=1/simulation_freq,
                                   env=env)

    compare_controller_parameters(nonlinear_mpc1, nonlinear_mpc2,
                                  nonlinear_mpc3, plot=plot,
                                  save_plot=save_plot)


if __name__ == '__main__':
    compare_parameters(plot=True, save_plot=False, loadmodel=False)
