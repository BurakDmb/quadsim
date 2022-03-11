import os
import numpy as np
import sys
sys.path.insert(0, './')

from src.controllers.pid import PID_Controller  # noqa: E402

from src.controllers.lqr import LQR  # noqa: E402
from src.controllers.lqg import LQG  # noqa: E402

from src.controllers.linear_mpc import Linear_MPC  # noqa: E402
from src.controllers.nonlinear_mpc import Nonlinear_MPC  # noqa: E402

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3 import SAC  # noqa: E402
from stable_baselines3 import A2C  # noqa: E402
from stable_baselines3 import TD3  # noqa: E402
from stable_baselines3 import DDPG  # noqa: E402


from src.envs.quad import DeterministicQuad  # noqa: E402
from src.envs.quad import StochasticQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402

from tests.constants import n_horizon, \
    control_freq, simulation_freq, t_end  # noqa: E402

from scripts.utils import compare_controllers  # noqa: E402


def experiment1_compare_all(compare_rl_models=False,
                            plot=False, save_plot=False, loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    # Env parameters
    dynamics_state = np.array([0, 0, 0, 0, 0, 0])
    custom_u_high = np.array([1, 1, 1])
    constant_reference = np.array([1, 0, 1, 0, 1, 0])
    set_custom_u_limit = False
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

    nonlinear_mpc = Nonlinear_MPC(t_end=t_end,
                                  n_horizon=int(n_horizon),
                                  c_step=1/control_freq,
                                  s_step=1/simulation_freq,
                                  env=env)
    linear_mpc = Linear_MPC(t_end=t_end,
                            n_horizon=int(n_horizon),
                            c_step=1/control_freq,
                            s_step=1/simulation_freq,
                            env=env)

    ddpg_model = ppo_model = sac_model = td3_model = a2c_model = None
    if compare_rl_models:
        # Controller 6
        ddpg_model = DDPG.load(
            os.getcwd() +
            "/results/2M_training/saves/ddpg-quad/ddpg-quad_end",
            device="cuda:0")
        # Controller 7
        ppo_model = PPO.load(
            os.getcwd() +
            "/results/2M_training/saves/ppo-quad/ppo-quad_end",
            device="cuda:0")
        # Controller 8
        sac_model = SAC.load(
            os.getcwd() +
            "/results/2M_training/saves/sac-quad/sac-quad_end",
            device="cuda:0")

        # Controller 9
        td3_model = TD3.load(
            os.getcwd() +
            "/results/2M_training/saves/td3-quad/td3-quad_end",
            device="cuda:0")
        # Controller 10
        a2c_model = A2C.load(
            os.getcwd() +
            "/results/2M_training/saves/a2c-quad/a2c-quad_end",
            device="cuda:0")

    compare_controllers(dynamics_state,
                        custom_u_high,
                        constant_reference,
                        set_custom_u_limit,
                        set_constant_reference,
                        pid,
                        lqr,
                        lqg,
                        nonlinear_mpc,
                        linear_mpc,
                        ddpg_model,
                        ppo_model,
                        sac_model,
                        td3_model,
                        a2c_model,
                        plot=plot,
                        save_plot=save_plot)


if __name__ == '__main__':
    experiment1_compare_all(compare_rl_models=False,
                            plot=True, save_plot=False, loadmodel=False)
