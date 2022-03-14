import os

from stable_baselines3 import TD3
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import DeterministicQuad
from quadsim.src.envs.quad import linear_quad_dynamics


def test_td3(plot=False, save_plot=False, loadmodel=False):
    if loadmodel:
        model = TD3.load(
            os.getcwd() +
            "/results/2M_training/saves/td3-quad/td3-quad_end",
            device="cuda:0")
    else:
        env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                                simulation_freq=250, control_freq=50,
                                keep_history=False)
        model = TD3('MlpPolicy', env, verbose=0,
                    device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/td3")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_td3(plot=True, save_plot=False, loadmodel=True)
