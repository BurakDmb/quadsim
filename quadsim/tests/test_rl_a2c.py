import os

from stable_baselines3 import A2C
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import Quad


def test_a2c(plot=False, save_plot=False, loadmodel=False):
    if loadmodel:
        model = A2C.load(
            os.getcwd() +
            "/results/2M_training/saves/a2c-quad/a2c-quad_end",
            device="cuda:0")
    else:
        env = Quad(
            is_linear=True, is_stochastic=False,
            t_end=t_end,
            simulation_freq=250,
            control_freq=250,
            keep_history=False)
        model = A2C('MlpPolicy', env, verbose=0,
                    device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/a2c")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_a2c(plot=True, save_plot=False, loadmodel=True)
