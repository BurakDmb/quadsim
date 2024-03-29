import unittest
import numpy as np


class TestEnvironments(unittest.TestCase):
    def test_process_rewardMinMaxLimit(self):
        # pip install multiprocess
        from multiprocess import Process, Queue

        def simulate(env, controller, num_episode, q):
            epi = 0
            while epi < num_episode:

                obs = env.reset()
                done = False
                while not done:
                    action = controller.predict(obs, deterministic=True)
                    if type(action) is tuple:
                        action = action[0]
                    obs, reward, done, _ = env.step(action)
                epi += 1
            env.serialize_casadi()
            q.put(env)
            return

        from quadsim.src.envs.quad import Quad

        class DummyController1:
            def predict(self, state_error, deterministic=True):
                return np.array([0.001, 0, 0])

        class DummyController2:
            def predict(self, state_error, deterministic=True):
                return np.array([0, 0.001, 0])

        class DummyController3:
            def predict(self, state_error, deterministic=True):
                return np.array([0, 0, 0.001])

        class DummyController4:
            def predict(self, state_error, deterministic=True):
                return np.array([0.001, 0.001, 0.001])

        control_freq = 200
        t_end = 5
        num_episode = 1

        controller1 = DummyController1()
        env_config1 = {
            'is_linear': True,
            'is_stochastic': False,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env1 = Quad(env_config1)

        controller2 = DummyController2()
        env_config2 = {
            'is_linear': True,
            'is_stochastic': False,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env2 = Quad(env_config2)

        controller3 = DummyController3()
        env_config3 = {
            'is_linear': True,
            'is_stochastic': False,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env3 = Quad(env_config3)

        controller4 = DummyController4()
        env_config4 = {
            'is_linear': True,
            'is_stochastic': False,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env4 = Quad(env_config4)

        q1 = Queue()
        q2 = Queue()
        q3 = Queue()
        q4 = Queue()

        p1 = Process(target=simulate, args=(env1, controller1,
                                            num_episode, q1))
        p2 = Process(target=simulate, args=(env2, controller2,
                                            num_episode, q2))
        p3 = Process(target=simulate, args=(env3, controller3,
                                            num_episode, q3))
        p4 = Process(target=simulate, args=(env4, controller4,
                                            num_episode, q4))
        p1.start()
        p2.start()
        p3.start()
        p4.start()
        env1 = q1.get()
        env2 = q2.get()
        env3 = q3.get()
        env4 = q4.get()
        p1.join()
        p2.join()
        p3.join()
        p4.join()

        evalRewardHist(self, env1, num_episode, t_end, control_freq)
        evalRewardHist(self, env2, num_episode, t_end, control_freq)
        evalRewardHist(self, env3, num_episode, t_end, control_freq)
        evalRewardHist(self, env4, num_episode, t_end, control_freq)

        evalSolutionHist(self, env1, num_episode, t_end, control_freq)
        evalSolutionHist(self, env2, num_episode, t_end, control_freq)
        evalSolutionHist(self, env3, num_episode, t_end, control_freq)
        evalSolutionHist(self, env4, num_episode, t_end, control_freq)

    def test_process_stochastic(self):
        # pip install multiprocess
        from multiprocess import Process, Queue

        def simulate(env, controller, num_episode, q):
            epi = 0
            while epi < num_episode:

                obs = env.reset()
                done = False
                while not done:
                    action = controller.predict(obs, deterministic=True)
                    if type(action) is tuple:
                        action = action[0]
                    obs, reward, done, _ = env.step(action)
                epi += 1
            env.serialize_casadi()
            q.put(env)
            return

        from quadsim.src.envs.quad import Quad

        class DummyController1:
            def predict(self, state_error, deterministic=True):
                return np.array([0.001, 0, 0])

        class DummyController2:
            def predict(self, state_error, deterministic=True):
                return np.array([0, 0.001, 0])

        class DummyController3:
            def predict(self, state_error, deterministic=True):
                return np.array([0, 0, 0.001])

        class DummyController4:
            def predict(self, state_error, deterministic=True):
                return np.array([0.001, 0.001, 0.001])

        control_freq = 200
        t_end = 5
        num_episode = 1

        controller1 = DummyController1()
        env_config1 = {
            'is_linear': True,
            'is_stochastic': True,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env1 = Quad(env_config1)

        controller2 = DummyController2()
        env_config2 = {
            'is_linear': True,
            'is_stochastic': True,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env2 = Quad(env_config2)

        controller3 = DummyController3()
        env_config3 = {
            'is_linear': True,
            'is_stochastic': True,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env3 = Quad(env_config3)

        controller4 = DummyController4()
        env_config4 = {
            'is_linear': True,
            'is_stochastic': True,
            't_end': t_end,
            'simulation_freq': 200,
            'control_freq': control_freq,
            'keep_history': True
        }
        env4 = Quad(env_config4)

        q1 = Queue()
        q2 = Queue()
        q3 = Queue()
        q4 = Queue()

        p1 = Process(target=simulate, args=(env1, controller1,
                                            num_episode, q1))
        p2 = Process(target=simulate, args=(env2, controller2,
                                            num_episode, q2))
        p3 = Process(target=simulate, args=(env3, controller3,
                                            num_episode, q3))
        p4 = Process(target=simulate, args=(env4, controller4,
                                            num_episode, q4))
        p1.start()
        p2.start()
        p3.start()
        p4.start()
        env1 = q1.get()
        env2 = q2.get()
        env3 = q3.get()
        env4 = q4.get()
        p1.join()
        p2.join()
        p3.join()
        p4.join()

        evalRewardHist(self, env1, num_episode, t_end, control_freq)
        evalRewardHist(self, env2, num_episode, t_end, control_freq)
        evalRewardHist(self, env3, num_episode, t_end, control_freq)
        evalRewardHist(self, env4, num_episode, t_end, control_freq)

        evalSolutionHist(self, env1, num_episode, t_end, control_freq)
        evalSolutionHist(self, env2, num_episode, t_end, control_freq)
        evalSolutionHist(self, env3, num_episode, t_end, control_freq)
        evalSolutionHist(self, env4, num_episode, t_end, control_freq)


def evalRewardHist(self_obj, env, num_episode, t_end, control_freq):
    rew_hist = env.history.sol_reward[:].T
    expected_hist_shape = (num_episode * t_end * control_freq, 1)
    expected_rew_hist_max = 0
    expected_rew_hist_min = -(np.sum(env.Q_coefficients) +
                              np.sum(env.R_coefficients))

    self_obj.assertEqual(expected_hist_shape, rew_hist.shape)
    self_obj.assertLessEqual(expected_rew_hist_min, rew_hist.min())
    self_obj.assertGreaterEqual(expected_rew_hist_max, rew_hist.max())


def evalSolutionHist(self_obj, env, num_episode, t_end, control_freq):
    sol_ref = env.history.sol_ref
    sol_x = env.history.sol_x

    expected_ref_shape = (6, num_episode * t_end * control_freq)
    expected_x_shape = (6, num_episode * t_end * control_freq)

    self_obj.assertGreaterEqual(
        env.high[0] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[0, :].max()), abs(sol_x[0, :].min()))))

    self_obj.assertGreaterEqual(
        env.high[1] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[1, :].max()), abs(sol_x[1, :].min()))))
    self_obj.assertGreaterEqual(
        env.high[2] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[2, :].max()), abs(sol_x[2, :].min()))))
    self_obj.assertGreaterEqual(
        env.high[3] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[3, :].max()), abs(sol_x[3, :].min()))))
    self_obj.assertGreaterEqual(
        env.high[4] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[4, :].max()), abs(sol_x[4, :].min()))))
    self_obj.assertGreaterEqual(
        env.high[5] + (
            num_episode * t_end * control_freq * env.noise_w_variance
            if hasattr(env, 'noise_w_variance') else 0),
        np.float32(max(abs(sol_x[5, :].max()), abs(sol_x[5, :].min()))))

    self_obj.assertEqual(expected_ref_shape, sol_ref.shape)
    self_obj.assertEqual(expected_x_shape, sol_x.shape)


def unittest_main(exit=False):
    print("*** Running unit tests ***")
    unittest.main(__name__, exit=exit)


if __name__ == '__main__':
    unittest_main(exit=True)
