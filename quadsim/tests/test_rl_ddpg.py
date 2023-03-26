import os

from stable_baselines3 import DDPG
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import Quad


def test_sac(plot=False, save_plot=False, loadmodel=False):
    if loadmodel:
        model = DDPG.load(
            os.getcwd() +
            "/results/2M_training/saves/ddpg-quad/ddpg-quad_end",
            device="cuda:0")
    else:
        env = Quad(
            is_linear=True, is_stochastic=False,
            t_end=t_end,
            simulation_freq=200,
            control_freq=200,
            keep_history=False)
        model = DDPG('MlpPolicy', env, verbose=0,
                     device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/ddpg")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_sac(plot=True, save_plot=False, loadmodel=True)
