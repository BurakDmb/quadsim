# import os
import numpy as np
import sys
sys.path.insert(0, './')

from src.controllers.pid import PID_Controller  # noqa: E402

from src.controllers.lqr import LQR  # noqa: E402
from src.controllers.lqg import LQG  # noqa: E402

from src.controllers.linear_mpc import Linear_MPC  # noqa: E402
from src.controllers.nonlinear_mpc import Nonlinear_MPC  # noqa: E402

# from stable_baselines3 import PPO  # noqa: E402
# from stable_baselines3 import SAC  # noqa: E402
# from stable_baselines3 import A2C  # noqa: E402
# from stable_baselines3 import TD3  # noqa: E402
# from stable_baselines3 import DDPG  # noqa: E402


from src.envs.quad import DeterministicQuad  # noqa: E402
from src.envs.quad import StochasticQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402

from tests.constants import n_horizon, \
    control_freq, simulation_freq, t_end  # noqa: E402

from scripts.utils import compare_controllers  # noqa: E402


def compare_controller_input_limits(plot=False, save_plot=False,
                                    loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    # Env parameters
    dynamics_state = np.array([0, 0, 0, 0, 0, 0])
    custom_u_high = np.array([5, 5, 5])
    constant_reference = np.array([1, 0, 1, 0, 1, 0])
    set_custom_u_limit = True
    set_constant_reference = True

    # Controller 1

    from tests.constants import rollKp, rollKi, rollKd, pitchKp, pitchKi,\
        pitchKd, yawKp, yawKi, yawKd,\
        T, limRoll, limPitch, limYaw  # noqa: E402

    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)
    # Controller 2

    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqr = LQR(env)

    # Controller 3
    env = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                         simulation_freq=250, control_freq=50,
                         keep_history=False)

    lqg = LQG(env)

    # Controller 4 and 5

    # start = time.time()
    # print("*** Function: ", sys._getframe().f_code.co_name, "***")
    nonlinear_mpc = Nonlinear_MPC(t_end=t_end,
                                  n_horizon=n_horizon,
                                  c_step=1/control_freq,
                                  s_step=1/simulation_freq,
                                  env=env)
    linear_mpc = Linear_MPC(t_end=t_end,
                            n_horizon=n_horizon,
                            c_step=1/control_freq,
                            s_step=1/simulation_freq,
                            env=env)

    compare_controllers(dynamics_state,
                        custom_u_high,
                        constant_reference,
                        set_custom_u_limit,
                        set_constant_reference,
                        pid, lqr, lqg, nonlinear_mpc, linear_mpc, plot=plot,
                        save_plot=save_plot)


if __name__ == '__main__':
    compare_controller_input_limits(plot=True, save_plot=False,
                                    loadmodel=False)
