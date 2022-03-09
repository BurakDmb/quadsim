import numpy as np
import time
import os

import sys
sys.path.insert(0, './')

from src.envs.quad import DeterministicQuad  # noqa: E402
from src.envs.quad import StochasticQuad  # noqa: E402
from src.envs.quad import linear_quad_dynamics  # noqa: E402
from src.envs.quad import nonlinear_quad_dynamics  # noqa: E402

from src.plotter import Plotter  # noqa: E402


# Note that before running, these LD Library Path needs to be configured
# Run this command in your terminal
# export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/src/controllers/hsl/lib"
def check_mpc_hsl_solver_in_path():
    if not any(
        os.getcwd()+"/src/controllers/hsl/lib" in x
            for x in os.environ['LD_LIBRARY_PATH'].split(':')):

        print("MPC HSL Solver in LD_LIBRARY_PATH does not found.")
        print(
            "Please run this command in terminal at " +
            "working directory root folder:"
            )
        print(
            r'export LD_LIBRARY_PATH=' +
            r'"$LD_LIBRARY_PATH:$(pwd)/src/controllers/hsl/lib"')
        exit(1)


def compare_controller_parameters(controller1, controller2, controller3,
                                  plot=False, save_plot=False):

    control_freq = 50
    simulation_freq = 250
    t_end = 5

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
    control_freq = 50
    simulation_freq = 250
    t_end = 5

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

    # TODO :Change Controller Names

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
        print("A2C Computation Time: %1.3f sec" % elapsed10)
        print("Env1: ")
        calculateControllerMetrics(c10_env1)
        print("Env3: ")
        calculateControllerMetrics(c10_env3)

    if plot:
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


def createEnvs(t_end, simulation_freq,
               control_freq, random_state_seed,
               set_custom_u_limit,
               custom_u_high,
               set_constant_reference,
               constant_reference,
               dynamics_state=np.array([3.14/4, 0, 0, 0, 0, 0]),
               eval_env=True):

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


def test_controller(controller, t_end, plot=False, save_plot=False,
                    constant_reference=None):
    control_freq = 50
    simulation_freq = 250
    t_end = 5

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
