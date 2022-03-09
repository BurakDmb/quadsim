import sys
import os
sys.path.insert(0, './')

from stable_baselines3 import PPO  # noqa: E402
from scripts.utils import test_controller  # noqa: E402
from tests.constants import t_end  # noqa: E402

from src.envs.quad import DeterministicQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402


def test_ppo(plot=False, save_plot=False, loadmodel=False):

    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    if loadmodel:
        # model = PPO.load("saves/ppo-quad/2021-10-21 01:21:42.065619",
        #                  device="cuda:0")
        model = PPO.load(
            os.getcwd() +
            "/results/2M_training/saves/ppo-quad/ppo-quad_700000_steps",
            device="cuda:0")
    else:
        model = PPO('MlpPolicy', env, verbose=1, device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/ppo")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_ppo(plot=True, save_plot=False, loadmodel=True)
