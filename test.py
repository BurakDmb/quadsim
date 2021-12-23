import numpy as np
import sys
import time

from envs.quad import StochasticQuad

n_horizon = 20
control_freq = 50
simulation_freq = 250
t_end = 5


def test_NonlinearMPC(plot=False, save_plot=False, loadmodel=False):
    from controllers.nonlinear_mpc import Nonlinear_MPC
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    import time

    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    start = time.time()
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    nonlinear_mpc = Nonlinear_MPC(t_end=t_end,
                                  n_horizon=n_horizon,
                                  c_step=1/control_freq,
                                  s_step=1/simulation_freq,
                                  env=env)
    test_controller(nonlinear_mpc, t_end, plot=plot, save_plot=save_plot)
    end = time.time()
    print(end-start)


def test_LinearMPC(plot=False, save_plot=False, loadmodel=False):
    from controllers.linear_mpc import Linear_MPC
    import time
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    start = time.time()
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    linear_mpc = Linear_MPC(t_end=t_end,
                            n_horizon=n_horizon,
                            c_step=1/control_freq,
                            s_step=1/simulation_freq,
                            env=env)
    test_controller(linear_mpc, t_end, plot=plot, save_plot=save_plot)
    end = time.time()
    print(end-start)


def test_lqr(plot=False, save_plot=False, loadmodel=False):
    from controllers.lqr import LQR
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    lqr = LQR(env)
    test_controller(lqr, t_end, plot=plot, save_plot=save_plot)

def test_lqg(plot=False, save_plot=False, loadmodel=False):
    from controllers.lqg import LQG
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    lqr = LQG(env)
    test_controller(lqr, t_end, plot=plot, save_plot=save_plot)


def test_rl(plot=False, save_plot=False, loadmodel=False):
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    from stable_baselines3 import PPO
    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)
    if loadmodel:
        # model = PPO.load("saves/ppo-quad/2021-10-21 01:21:42.065619",
        #                  device="cuda:0")
        model = PPO.load("saves/ppo-quad/ppo-quad_700000_steps",
                         device="cuda:0")
    else:
        model = PPO('MlpPolicy', env, verbose=1, device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/ppo")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


def test_sac(plot=False, save_plot=False, loadmodel=False):
    # from parameters import sac_learning_setting as learning_setting
    # from envs.quad import DeterministicQuad, linear_quad_dynamics
    from stable_baselines3 import SAC

    if loadmodel:
        # model = SAC.load("saves/sac-quad/_end_2021-10-22 03:18:15.143412",
        #                  device="cuda:0")
        model = SAC.load("saves/sac-quad/sac-quad_550000_steps",
                         device="cuda:0")
    else:
        from envs.quad import DeterministicQuad, linear_quad_dynamics
        env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                                simulation_freq=250, control_freq=50,
                                keep_history=False)
        model = SAC('MlpPolicy', env, verbose=0,
                    device='cuda:0')
        model.learn(total_timesteps=100_000)
        model.save("rl_models/ppo")

    test_controller(model, t_end, plot=plot, save_plot=save_plot)


def test_pid(plot=False, save_plot=False):
    from controllers.pid import PID_Controller

    rollKp = 4.72531916175911
    rollKi = 3.73086282471387
    rollKd = 1.49621161575424
    pitchKp = 3.63871317561002
    pitchKi = 2.14232438611662
    pitchKd = 1.54507805402352
    yawKp = 4.6284037687056
    yawKi = 2.72501342753779
    yawKd = 1.96532255856848

    T = 1.0/50
    limRoll = 1.2568
    limPitch = 1.2568
    limYaw = 0.2145

    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)

    test_controller(pid, t_end, plot=plot, save_plot=save_plot)


def compare_all(compare_rl_models=False,
                plot=False, save_plot=False, loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    # Env parameters
    dynamics_state = np.array([0, 0, 0, 0, 0, 0])
    custom_u_high = np.array([1, 1, 1])
    constant_reference = np.array([1, 0, 1, 0, 1, 0])
    set_custom_u_limit = False
    set_constant_reference = True

    # Controller 1
    from controllers.pid import PID_Controller

    rollKp = 4.72531916175911
    rollKi = 3.73086282471387
    rollKd = 1.49621161575424
    pitchKp = 3.63871317561002
    pitchKi = 2.14232438611662
    pitchKd = 1.54507805402352
    yawKp = 4.6284037687056
    yawKi = 2.72501342753779
    yawKd = 1.96532255856848

    T = 1.0/50
    limRoll = 1.2568
    limPitch = 1.2568
    limYaw = 0.2145

    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)
    # Controller 2
    from controllers.lqr import LQR
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqr = LQR(env)

    # Controller 3
    from controllers.lqg import LQG
    from envs.quad import StochasticQuad, linear_quad_dynamics
    env = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqg = LQG(env)

    # Controller 4 and 5
    from controllers.nonlinear_mpc import Nonlinear_MPC
    from controllers.linear_mpc import Linear_MPC

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
        from stable_baselines3 import DDPG
        ddpg_model = DDPG.load(
            "results/2M_training/saves/ddpg-quad/ddpg-quad_end",
            device="cuda:0")
        # Controller 7
        from stable_baselines3 import PPO
        ppo_model = PPO.load(
            "results/2M_training/saves/ppo-quad/ppo-quad_end",
            device="cuda:0")
        # Controller 8
        from stable_baselines3 import SAC
        sac_model = SAC.load(
            "results/2M_training/saves/sac-quad/sac-quad_end",
            device="cuda:0")

        # Controller 9
        from stable_baselines3 import TD3
        td3_model = TD3.load(
            "results/2M_training/saves/td3-quad/td3-quad_end",
            device="cuda:0")
        # Controller 10
        from stable_baselines3 import A2C
        a2c_model = A2C.load(
            "results/2M_training/saves/a2c-quad/a2c-quad_end",
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
    from controllers.pid import PID_Controller

    rollKp = 4.72531916175911
    rollKi = 3.73086282471387
    rollKd = 1.49621161575424
    pitchKp = 3.63871317561002
    pitchKi = 2.14232438611662
    pitchKd = 1.54507805402352
    yawKp = 4.6284037687056
    yawKi = 2.72501342753779
    yawKd = 1.96532255856848

    T = 1.0/50
    limRoll = 1.2568
    limPitch = 1.2568
    limYaw = 0.2145

    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)
    # Controller 2
    from controllers.lqr import LQR
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqr = LQR(env)
    
    # Controller 3
    from controllers.lqg import LQG
    from envs.quad import StochasticQuad, linear_quad_dynamics
    env = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqg = LQG(env)

    # Controller 4 and 5
    from controllers.nonlinear_mpc import Nonlinear_MPC
    from controllers.linear_mpc import Linear_MPC
    # import time

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


def compare_initial_conditions(plot=False, save_plot=False, loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")

    # Env parameters
    dynamics_state = np.array([3.14/4, 0, 0, 0, 0, 0])
    constant_reference = np.array([0, 0, 0, 0, 0, 0])
    custom_u_high = np.array([1, 1, 1])
    set_custom_u_limit = False
    set_constant_reference = True

    # Controller 1
    from controllers.pid import PID_Controller

    rollKp = 4.72531916175911
    rollKi = 3.73086282471387
    rollKd = 1.49621161575424
    pitchKp = 3.63871317561002
    pitchKi = 2.14232438611662
    pitchKd = 1.54507805402352
    yawKp = 4.6284037687056
    yawKi = 2.72501342753779
    yawKd = 1.96532255856848

    T = 1.0/50
    limRoll = 1.2568
    limPitch = 1.2568
    limYaw = 0.2145

    pid = PID_Controller(rollKp, rollKi, rollKd,
                         pitchKp, pitchKi, pitchKd,
                         yawKp, yawKi, yawKd, T,
                         limRoll, limPitch, limYaw)
    # Controller 2
    from controllers.lqr import LQR
    from envs.quad import DeterministicQuad, linear_quad_dynamics
    env = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqr = LQR(env)

    # Controller 3
    from controllers.lqg import LQG
    from envs.quad import StochasticQuad, linear_quad_dynamics
    env = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                            simulation_freq=250, control_freq=50,
                            keep_history=False)

    lqg = LQG(env)

    # Controller 4-5
    from controllers.nonlinear_mpc import Nonlinear_MPC
    from controllers.linear_mpc import Linear_MPC
    # import time

    # start = time.time()
    # print("*** Function: ", sys._getframe().f_code.co_name, "***")
    nonlinear_mpc = Nonlinear_MPC(t_end=t_end,
                                  n_horizon=n_horizon,
                                  c_step=1/control_freq,
                                  s_step=1/simulation_freq,
                                  env=env)

    linear_mpc = Linear_MPC(t_end=t_end,
                            n_horizon=int(n_horizon),
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


def compare_parameters(plot=False, save_plot=False, loadmodel=False):
    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    from controllers.nonlinear_mpc import Nonlinear_MPC
    from envs.quad import DeterministicQuad, linear_quad_dynamics
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


def test_all_environments_open_loop(plot=False, save_plot=False):
    class DummyController:
        def __init__(self):
            pass

        # State is the 6 dimensional vector(6,) which holds the
        # reference minus the current state information
        def predict(self, state_error, deterministic=True):
            return np.array([0, 0, 0])

    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    t_end = 5
    dummyController = DummyController()
    test_controller(dummyController, t_end, plot=plot, save_plot=save_plot)


def createEnvs(t_end, simulation_freq,
               control_freq, random_state_seed,
               set_custom_u_limit,
               custom_u_high,
               set_constant_reference,
               constant_reference,
               dynamics_state=np.array([3.14/4, 0, 0, 0, 0, 0]),
               eval_env=True):
    from envs.quad import DeterministicQuad, StochasticQuad
    from envs.quad import linear_quad_dynamics, nonlinear_quad_dynamics

    # Linear deterministic quadcopter
    env1 = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                             simulation_freq=simulation_freq,
                             control_freq=control_freq, random_state_seed=0,
                             dynamics_state=dynamics_state,
                             set_custom_u_limit=set_custom_u_limit,
                             custom_u_high=custom_u_high,
                             set_constant_reference=True,
                             constant_reference=constant_reference,
                             eval_env=eval_env)
    # Linear stochastic quadcopter
    env2 = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                          simulation_freq=simulation_freq,
                          control_freq=control_freq, random_state_seed=0,
                          dynamics_state=dynamics_state,
                          set_custom_u_limit=set_custom_u_limit,
                          custom_u_high=custom_u_high,
                          set_constant_reference=True,
                          constant_reference=constant_reference,
                          eval_env=eval_env)
    # Nonlinear deterministic quadcopter
    env3 = DeterministicQuad(nonlinear_quad_dynamics, t_end=t_end,
                             simulation_freq=simulation_freq,
                             control_freq=control_freq, random_state_seed=0,
                             dynamics_state=dynamics_state,
                             set_custom_u_limit=set_custom_u_limit,
                             custom_u_high=custom_u_high,
                             set_constant_reference=True,
                             constant_reference=constant_reference,
                             eval_env=eval_env)
    # Nonlinear stochastic quadcopter
    env4 = StochasticQuad(nonlinear_quad_dynamics, t_end=t_end,
                          simulation_freq=simulation_freq,
                          control_freq=control_freq, random_state_seed=0,
                          dynamics_state=dynamics_state,
                          set_custom_u_limit=set_custom_u_limit,
                          custom_u_high=custom_u_high,
                          set_constant_reference=True,
                          constant_reference=constant_reference,
                          eval_env=eval_env)
    return env1, env2, env3, env4


def compare_controllers(dynamics_state,
                        custom_u_high,
                        constant_reference,
                        set_custom_u_limit,
                        set_constant_reference,
                        controller1,
                        controller2,
                        controller3,
                        controller4,
                        controller5=None,
                        controller6=None,
                        controller7=None,
                        controller8=None,
                        controller9=None,
                        controller10=None,
                        plot=False, save_plot=False):
    c1_env1 = c1_env2 = \
        c1_env3 = c1_env4 = \
        c2_env1 = c2_env2 = \
        c2_env3 = c2_env4 = \
        c3_env1 = c3_env2 = \
        c3_env3 = c3_env4 = \
        c4_env1 = c4_env2 = \
        c4_env3 = c4_env4 = \
        c5_env1 = c5_env2 = \
        c5_env3 = c5_env4 = \
        c6_env1 = c6_env2 = \
        c6_env3 = c6_env4 = \
        c7_env1 = c7_env2 = \
        c7_env3 = c7_env4 = \
        c8_env1 = c8_env2 = \
        c8_env3 = c8_env4 = \
        c9_env1 = c9_env2 = \
        c9_env3 = c9_env4 = \
        c10_env1 = c10_env2 = \
        c10_env3 = c10_env4 = None

    num_episode = 1

    if controller1 is not None:
        c1_env1, c1_env2, c1_env3, c1_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t1 = time.time()
        simulate_envs(controller1, c1_env1, c1_env2,
                      c1_env3, c1_env4, num_episode)
        elapsed1 = time.time() - t1
        print("PID Computation Time: %1.3f sec" % elapsed1)
        print("Env1: ")
        calculateControllerMetrics(c1_env1)
        print("Env3: ")
        calculateControllerMetrics(c1_env3)

    if controller2 is not None:
        c2_env1, c2_env2, c2_env3, c2_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t2 = time.time()
        simulate_envs(controller2, c2_env1, c2_env2,
                      c2_env3, c2_env4, num_episode)
        elapsed2 = time.time() - t2
        print("LQR Computation Time: %1.3f sec" % elapsed2)
        print("Env1: ")
        calculateControllerMetrics(c2_env1)
        print("Env3: ")
        calculateControllerMetrics(c2_env3)

    #TODO :Change Controller Names

    if controller3 is not None:
        c3_env1, c3_env2, c3_env3, c3_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t3 = time.time()
        simulate_envs(controller3, c3_env1, c3_env2,
                      c3_env3, c3_env4, num_episode)
        elapsed3 = time.time() - t3
        print("LQG Computation Time: %1.3f sec" % elapsed3)
        print("Env1: ")
        calculateControllerMetrics(c3_env1)
        print("Env3: ")
        calculateControllerMetrics(c3_env3)

    if controller4 is not None:
        c4_env1, c4_env2, c4_env3, c4_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t4 = time.time()
        simulate_envs(controller4, c4_env1, c4_env2,
                      c4_env3, c4_env4, num_episode)
        elapsed4 = time.time() - t4
        print("NL-MPC Computation Time: %1.3f sec" % elapsed4)
        print("Env1: ")
        calculateControllerMetrics(c4_env1)
        print("Env3: ")
        calculateControllerMetrics(c4_env3)

    if controller5 is not None:
        c5_env1, c5_env2, c5_env3, c5_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t5 = time.time()
        simulate_envs(controller5, c5_env1, c5_env2,
                      c5_env3, c5_env4, num_episode)
        elapsed5 = time.time() - t5
        print("L-MPC Computation Time: %1.3f sec" % elapsed5)
        print("Env1: ")
        calculateControllerMetrics(c5_env1)
        print("Env3: ")
        calculateControllerMetrics(c5_env3)

    if controller6 is not None:
        c6_env1, c6_env2, c6_env3, c6_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t6 = time.time()
        simulate_envs(controller6, c6_env1, c6_env2,
                      c6_env3, c6_env4, num_episode)
        elapsed6 = time.time() - t6
        print("DDPG Computation Time: %1.3f sec" % elapsed6)
        print("Env1: ")
        calculateControllerMetrics(c6_env1)
        print("Env3: ")
        calculateControllerMetrics(c6_env3)

    if controller7 is not None:
        c7_env1, c7_env2, c7_env3, c7_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t7 = time.time()
        simulate_envs(controller7, c7_env1, c7_env2,
                      c7_env3, c7_env4, num_episode)
        elapsed7 = time.time() - t7
        print("PPO Computation Time: %1.3f sec" % elapsed7)
        print("Env1: ")
        calculateControllerMetrics(c7_env1)
        print("Env3: ")
        calculateControllerMetrics(c7_env3)

    if controller8 is not None:
        c8_env1, c8_env2, c8_env3, c8_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t8 = time.time()
        simulate_envs(controller8, c8_env1, c8_env2,
                      c8_env3, c8_env4, num_episode)
        elapsed8 = time.time() - t8
        print("SAC Computation Time: %1.3f sec" % elapsed8)
        print("Env1: ")
        calculateControllerMetrics(c8_env1)
        print("Env3: ")
        calculateControllerMetrics(c8_env3)

    if controller9 is not None:
        c9_env1, c9_env2, c9_env3, c9_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t9 = time.time()
        simulate_envs(controller9, c9_env1, c9_env2,
                      c9_env3, c9_env4, num_episode)
        elapsed9 = time.time() - t9
        print("TD3 A2C Computation Time: %1.3f sec" % elapsed9)
        print("Env1: ")
        calculateControllerMetrics(c9_env1)
        print("Env3: ")
        calculateControllerMetrics(c9_env3)

    if controller10 is not None:
        c10_env1, c10_env2, c10_env3, c10_env4 =\
            createEnvs(t_end=t_end,
                       simulation_freq=simulation_freq,
                       control_freq=control_freq,
                       random_state_seed=0,
                       set_custom_u_limit=set_custom_u_limit,
                       custom_u_high=custom_u_high,
                       set_constant_reference=set_constant_reference,
                       constant_reference=constant_reference,
                       dynamics_state=dynamics_state)
        t10 = time.time()
        simulate_envs(controller10, c10_env1, c10_env2,
                      c10_env3, c10_env4, num_episode)
        elapsed10 = time.time() - t10
        print("A2C Computation Time: %1.3f sec" % elapsed9)
        print("Env1: ")
        calculateControllerMetrics(c10_env1)
        print("Env3: ")
        calculateControllerMetrics(c10_env3)

    if plot:
        from plotter import Plotter
        plotter = Plotter(type(controller1).__name__ +
                          '-' + type(controller2).__name__ +
                          '-' + type(controller3).__name__)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=0)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=1)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=2)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=3)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=4)
        plotter.plot_only_specific_element(
                                           c1_env1, c1_env2,
                                           c1_env3, c1_env4,
                                           c2_env1, c2_env2,
                                           c2_env3, c2_env4,
                                           c3_env1, c3_env2,
                                           c3_env3, c3_env4,
                                           c4_env1, c4_env2,
                                           c4_env3, c4_env4,
                                           c5_env1, c5_env2,
                                           c5_env3, c5_env4,
                                           c6_env1, c6_env2,
                                           c6_env3, c6_env4,
                                           c7_env1, c7_env2,
                                           c7_env3, c7_env4,
                                           c8_env1, c8_env2,
                                           c8_env3, c8_env4,
                                           c9_env1, c9_env2,
                                           c9_env3, c9_env4,
                                           c10_env1, c10_env2,
                                           c10_env3, c10_env4,
                                           save_plot=save_plot, axis=5)
        plotter.plot_compare_input_values(
                                          c1_env1, c1_env2,
                                          c1_env3, c1_env4,
                                          c2_env1, c2_env2,
                                          c2_env3, c2_env4,
                                          c3_env1, c3_env2,
                                          c3_env3, c3_env4,
                                          c4_env1, c4_env2,
                                          c4_env3, c4_env4,
                                          c5_env1, c5_env2,
                                          c5_env3, c5_env4,
                                          c6_env1, c6_env2,
                                          c6_env3, c6_env4,
                                          c7_env1, c7_env2,
                                          c7_env3, c7_env4,
                                          c8_env1, c8_env2,
                                          c8_env3, c8_env4,
                                          c9_env1, c9_env2,
                                          c9_env3, c9_env4,
                                          c10_env1, c10_env2,
                                          c10_env3, c10_env4,
                                          save_plot=save_plot)
        plotter.plot_compare_input_values(
                                          c1_env1, c1_env2,
                                          c1_env3, c1_env4,
                                          c2_env1, c2_env2,
                                          c2_env3, c2_env4,
                                          c3_env1, c3_env2,
                                          c3_env3, c3_env4,
                                          c4_env1, c4_env2,
                                          c4_env3, c4_env4,
                                          c5_env1, c5_env2,
                                          c5_env3, c5_env4,
                                          c6_env1, c6_env2,
                                          c6_env3, c6_env4,
                                          c7_env1, c7_env2,
                                          c7_env3, c7_env4,
                                          c8_env1, c8_env2,
                                          c8_env3, c8_env4,
                                          c9_env1, c9_env2,
                                          c9_env3, c9_env4,
                                          c10_env1, c10_env2,
                                          c10_env3, c10_env4,
                                          save_plot=save_plot, axis=1)
        plotter.plot_compare_input_values(
                                          c1_env1, c1_env2,
                                          c1_env3, c1_env4,
                                          c2_env1, c2_env2,
                                          c2_env3, c2_env4,
                                          c3_env1, c3_env2,
                                          c3_env3, c3_env4,
                                          c4_env1, c4_env2,
                                          c4_env3, c4_env4,
                                          c5_env1, c5_env2,
                                          c5_env3, c5_env4,
                                          c6_env1, c6_env2,
                                          c6_env3, c6_env4,
                                          c7_env1, c7_env2,
                                          c7_env3, c7_env4,
                                          c8_env1, c8_env2,
                                          c8_env3, c8_env4,
                                          c9_env1, c9_env2,
                                          c9_env3, c9_env4,
                                          c10_env1, c10_env2,
                                          c10_env3, c10_env4,
                                          save_plot=save_plot, axis=2)
        if plot:
            plotter.show()


def compare_controller_parameters(controller1, controller2, controller3,
                                  plot=False, save_plot=False):

    constant_reference = np.array([1, 0, 1, 0, 1, 0])
    custom_u_high = np.array([1, 1, 1])
    set_custom_u_limit = False
    set_constant_reference = True

    c1_env1, c1_env2, c1_env3, c1_env4 =\
        createEnvs(t_end=t_end,
                   simulation_freq=simulation_freq,
                   control_freq=control_freq,
                   random_state_seed=0,
                   set_custom_u_limit=set_custom_u_limit,
                   custom_u_high=custom_u_high,
                   set_constant_reference=set_constant_reference,
                   constant_reference=constant_reference)

    c2_env1, c2_env2, c2_env3, c2_env4 =\
        createEnvs(t_end=t_end,
                   simulation_freq=simulation_freq,
                   control_freq=control_freq,
                   random_state_seed=0,
                   set_custom_u_limit=set_custom_u_limit,
                   custom_u_high=custom_u_high,
                   set_constant_reference=set_constant_reference,
                   constant_reference=constant_reference)

    c3_env1, c3_env2, c3_env3, c3_env4 =\
        createEnvs(t_end=t_end,
                   simulation_freq=simulation_freq,
                   control_freq=control_freq,
                   random_state_seed=0,
                   set_custom_u_limit=set_custom_u_limit,
                   custom_u_high=custom_u_high,
                   set_constant_reference=set_constant_reference,
                   constant_reference=constant_reference)

    num_episode = 1
    t1 = time.time()
    simulate_envs(controller1, c1_env1, c1_env2, c1_env3, c1_env4, num_episode)
    elapsed1 = time.time() - t1
    print("Prediction Horizon=10")
    print("Computation Time: %1.3f sec" % elapsed1)
    calculateControllerMetrics(c1_env1)
    t2 = time.time()
    simulate_envs(controller2, c2_env1, c2_env2, c2_env3, c2_env4, num_episode)
    elapsed2 = time.time() - t2
    print("Prediction Horizon=20")
    print("Computation Time: %1.3f sec" % elapsed2)
    calculateControllerMetrics(c2_env1)
    t3 = time.time()
    simulate_envs(controller3, c3_env1, c3_env2, c3_env3, c3_env4, num_episode)
    elapsed3 = time.time() - t3
    print("Prediction Horizon=100")
    print("Computation Time: %1.3f sec" % elapsed3)
    calculateControllerMetrics(c3_env1)

    if plot:
        from plotter import Plotter
        plotter = Plotter(type(controller1).__name__ +
                          '-' + type(controller2).__name__ +
                          '-' + type(controller3).__name__)
        plotter.plot_compare_parameters(
                                           c1_env1, c1_env2, c1_env3, c1_env4,
                                           c2_env1, c2_env2, c2_env3, c2_env4,
                                           c3_env1, c3_env2, c3_env3, c3_env4,
                                           save_plot=save_plot)
        if plot:
            plotter.show()


def test_controller(controller, t_end, plot=False, save_plot=False,
                    constant_reference=None):

    if constant_reference is None:
        constant_reference = np.array([1, 0, 1, 0, 1, 0])
    custom_u_high = np.array([0.1, 0.1, 0.1])

    env1, env2, env3, env4 =\
        createEnvs(t_end=t_end,
                   simulation_freq=simulation_freq,
                   control_freq=control_freq,
                   random_state_seed=0,
                   set_custom_u_limit=False,
                   custom_u_high=custom_u_high,
                   set_constant_reference=True,
                   constant_reference=constant_reference)

    num_episode = 1
    simulate_envs(controller, env1, env2, env3, env4, num_episode)
    calculateControllerMetrics(env1)

    if plot:
        from plotter import Plotter
        plotter = Plotter(type(controller).__name__)
        plotter.plot_only_specific_element(env1, env2, env3, env4,
                                           save_plot=save_plot)
        plotter.plot_only_specific_element(env1, env2, env3, env4,
                                           save_plot=save_plot, axis=1)
        plotter.plot_all_with_reference(env1, env2, env3, env4,
                                        save_plot=save_plot)
        plotter.plot_reward(env1, env2, env3, env4, save_plot=save_plot)
        plotter.plot_actions(env1, env2, env3, env4, save_plot=save_plot)
        if plot:
            plotter.show()
    return env1, env2, env3, env4


def calculateControllerMetrics(env):
    resp_final = env.history.sol_x[0][-1]
    tol = 5e-2
    tl_ind = 0
    th_ind = 0
    ts_ind = 0
    tp = 0
    vl = 0.10*resp_final
    vh = 0.90*resp_final
    vs_l = 0.98*resp_final
    vs_h = 1.02*resp_final
    peak_val = 0
    for ind, val in enumerate(env.history.sol_x[0]):
        if abs(val-vl) < tol:
            tl_ind = ind
            break
    for ind, val in enumerate(env.history.sol_x[0]):
        if abs(val-vh) < tol:
            th_ind = ind
            break
    for ind, val in enumerate(env.history.sol_x[0]):
        if ts_ind != 0:
            if vs_l > val or vs_h < val:
                ts_ind = 0
        if vs_l < val and vs_h > val and ts_ind == 0:
            ts_ind = ind
        if val > peak_val:
            peak_val = val
            tp = ind
    rise_time = env.history.sol_t[0][th_ind]-env.history.sol_t[0][tl_ind]
    settling_time = env.history.sol_t[0][ts_ind]
    overshoot = (peak_val-resp_final)*100/resp_final
    peak = peak_val
    peak_time = env.history.sol_t[0][tp]
    ss_error = abs(env.reference_state[0] - resp_final)
    total_rew = env.history.sol_reward.sum()

    print("Env-Shape: ", env.t.shape, env.history.sol_ref.shape,
          env.history.sol_x.shape, env.history.sol_reward.shape)
    print("Rise time: %1.3f sec" % rise_time)
    print("Settling time: %1.3f sec" % settling_time)
    print("Overshoot: %2.3f percent" % overshoot)
    print("Peak time: %1.3f sec" % peak_time)
    print("Steady State Error: %2.3f rad" % ss_error)
    print("Total Reward: %2.3f" % total_rew)
    print("---------------------------------------------------------------")
    print("---------------------------------------------------------------")
    return rise_time, settling_time, overshoot, peak, peak_time


def simulate_envs(controller, env1, env2, env3, env4, num_episode):
    for epi in range(num_episode):
        obs = env1.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if type(action) is tuple:
                action = action[0]
            obs, reward, done, _ = env1.step(action)

        obs = env2.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if type(action) is tuple:
                action = action[0]
            obs, reward, done, _ = env2.step(action)

        obs = env3.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if type(action) is tuple:
                action = action[0]
            obs, reward, done, _ = env3.step(action)

        obs = env4.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if type(action) is tuple:
                action = action[0]
            obs, reward, done, _ = env4.step(action)


if __name__ == '__main__':
    from unit_tests import unittest_main
    # unittest_main()

    # test_all_environments_open_loop(plot=True, save_plot=False)
    # test_pid(plot=True, save_plot=False)
    # test_lqr(plot=True, save_plot=False, loadmodel=False)
    # test_lqg(plot=True, save_plot=False, loadmodel=False)
    # test_NonlinearMPC(plot=True, save_plot=False)
    # test_LinearMPC(plot=True, save_plot=False)
    # test_rl(plot=True, save_plot=False, loadmodel=True)
    # test_sac(plot=True, save_plot=False, loadmodel=True)

    compare_all(compare_rl_models=False,
                plot=True, save_plot=False, loadmodel=False)
    # compare_controller_input_limits(plot=True, save_plot=False,
    #                                 loadmodel=False)
    # compare_parameters(plot=True, save_plot=False, loadmodel=False)
    # compare_initial_conditions(plot=True, save_plot=False, loadmodel=False)
