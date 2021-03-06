import os

from stable_baselines3 import SAC
from quadsim.scripts.utils import test_controller
from quadsim.tests.constants import t_end

from quadsim.src.envs.quad import DeterministicQuad
from quadsim.src.envs.quad import linear_quad_dynamics


def test_sac(plot=False, save_plot=False, loadmodel=False):
    if loadmodel:
        model = SAC.load(
            os.getcwd() +
            "/results/2M_training/saves/sac-quad/ppo-quad_end",
            device="cuda:0")
    else:
        env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                                simulation_freq=250, control_freq=50,
                                keep_history=False)
        model = SAC('MlpPolicy', env, verbose=0,
                    device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/sac")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


if __name__ == "__main__":
    test_sac(plot=True, save_plot=False, loadmodel=True)
