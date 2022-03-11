import sys
import os
sys.path.insert(0, './')

from stable_baselines3 import SAC  # noqa: E402
from scripts.utils import test_controller  # noqa: E402
from tests.constants import t_end  # noqa: E402

from src.envs.quad import DeterministicQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402


def test_sac(plot=False, save_plot=False, loadmodel=False):
    if loadmodel:
        # model = SAC.load("saves/sac-quad/_end_2021-10-22 03:18:15.143412",
        #                  device="cuda:0")
        model = SAC.load(
            os.getcwd() +
            "/results/2M_training/saves/sac-quad/sac-quad_550000_steps",
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
