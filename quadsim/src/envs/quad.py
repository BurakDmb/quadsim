import gymnasium as gym
import gym as oldgym
from gymnasium import spaces
import numpy as np
from scipy.integrate import solve_ivp
from quadsim.src.envs.util_history import History
import casadi
import sys


initializate_inertia_values = np.array(
    [0.0213, 0.02217, 0.0282],  # kgm^2
    dtype=np.float32)
m = 1.587  # kg
g = 9.81  # ms^-2
mg = m*g
d = 0.243  # m
b = 4.0687e-7/((2*np.pi/60)**2)  # kgm
k = 8.4367e-9/((2*np.pi/60)**2)  # kgm^2

db = d*b
w_max = 4720 * (2*np.pi/60)  # rad/s

# motor min and max speed values, max=4720 and min=3093 rpm
# by the reference (converted into rad/s values)
# the minimum speed value will be calculated from
# the given mass, g and b value.
w_max = w_max
w_min = np.rint(np.sqrt((m * g)/(4*b)))

u1_max = u2_max = d*b*((w_max**2)-(w_min**2))
u3_max = k*2*((w_max**2)-(w_min**2))
u_min = 0.05

# Experimental soft limits.
# Defining soft limits for physical life quadcopter
# Theoretical maximum angular speeds are 2000deg/s for the imu.
# But for experimental soft limits, below values are used.

# Below the link for the IMU used in pixhawk 2.1 cube
# https://ardupilot.org/copter/docs/common-thecube-overview.html
# https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/

soft_ref_diff_phi = 2*np.pi
soft_ref_diff_phidot = 2*np.pi
soft_ref_diff_theta = 2*np.pi
soft_ref_diff_thetadot = 2*np.pi
soft_ref_diff_psi = 2*np.pi
soft_ref_diff_psidot = 2*np.pi

soft_phi = np.pi
soft_phidot = 2*np.pi
soft_theta = np.pi
soft_thetadot = 2*np.pi
soft_psi = np.pi
soft_psidot = 2*np.pi

random_ref_val = np.pi/6


# Higher the coefficient, higher the importance and the effect.
Q_coefficients = np.array([20.0, 1.125, 20.0, 1.125, 2.0, 1.125])
R_coefficients = np.array([0.2, 0.2, 0.2])
delta_u_coefficient = 0.0


# Vectorized dynamics function usage:
# y will be in shape (n,k)
# output will be in shape (n,k)
# each column corresponds to a single column in y.
def linear_quad_dynamics(
        t, x,
        A, B, G, Ixx, Iyy, Izz,
        u1=0, u2=0, u3=0, wk=None):

    # u = [db*(w4s - w2s), db*(w1s - w3s), k*(w1s + w3s - w4s - w2s)]
    # u = [db*(w4*w4 - w2*w2), db*(w1*w1 - w3*w3),
    #      k*(w1*w1 + w3*w3 - w4*w4 - w2*w2)]
    u = [u1, u2, u3]
    if x.ndim == 2:
        x_shape = x.shape[1]
        U = np.array([u, ]*x_shape).transpose()
    else:
        U = np.array(u)
    dxdt = np.dot(A, x) + np.dot(B, U)
    if wk is not None:
        wk_ = np.array(wk)
        if wk_.ndim == 2:
            dxdt += np.dot(G, wk_)
        else:
            dxdt += np.dot(G, wk_[:, np.newaxis])
    return dxdt


# Vectorized dynamics function usage:
# y will be in shape (n,k)
# output will be in shape (n,k)
# each column corresponds to a single column in y.
# for quadcopter dynamics, n=6, which are
# phidot, phidotdot, thetadot, thetadotdot, psidot, psidotdot.
def nonlinear_quad_dynamics(
        t, x,
        A, B, G, Ixx, Iyy, Izz,
        u1=0, u2=0, u3=0, wk=None):

    # u = [db*(w4s - w2s), db*(w1s - w3s), k*(w1s + w3s - w4s - w2s)]
    u = [u1, u2, u3]
    if x.ndim == 2:
        x_shape = x.shape[1]
        U = np.array([u, ]*x_shape).transpose()
    else:
        U = np.array(u)

    phidot = x[1, :]
    thetadot = x[3, :]
    psidot = x[5, :]
    phidotdot = (np.dot(
        (Iyy-Izz),
        np.multiply(thetadot, psidot)) + (U[0, :])*(u1_max))/Ixx
    thetadotdot = (np.dot(
        (Izz-Ixx),
        np.multiply(phidot, psidot)) + (U[1, :])*(u2_max))/Iyy
    psidotdot = (np.dot(
        (Ixx-Iyy),
        np.multiply(phidot, thetadot)) + (U[2, :])*(u3_max))/Izz

    dxdt = np.array([x[1, :], phidotdot, x[3, :],
                    thetadotdot, x[5, :], psidotdot])

    if wk is not None:
        dxdt += np.dot(G, wk)
    return dxdt


class Quad(gym.Env):
    """
    Configurable Linear/Nonlinear and Deterministic/Stochastic
    Quadcopter System Dynamics
    System Input(actions): U(U1, U2, U3) torque values
    System Output(states): X = phi, phidot, theta, thetadot, psi, psidot
    """

    def __init__(self, env_config, **kwargs):
        is_linear = env_config.get(
            'is_linear', True)
        is_stochastic = env_config.get(
            'is_stochastic', False)
        t_start = env_config.get(
            't_start', 0.0)
        t_end = env_config.get(
            't_end', 3.0)
        simulation_freq = env_config.get(
            'simulation_freq', 200)
        control_freq = env_config.get(
            'control_freq', 200)
        dynamics_state = env_config.get(
            'dynamics_state', np.array([0., 0., 0., 0., 0., 0.]))
        noise_w_mean = env_config.get(
            'noise_w_mean', 0)
        noise_w_variance = env_config.get(
            'noise_w_variance', 0.02)
        noise_v_mean = env_config.get(
            'noise_v_mean', 0)
        noise_v_variance = env_config.get(
            'noise_v_variance', 0.02)
        keep_history = env_config.get(
            'keep_history', True)
        random_state_seed = env_config.get(
            'random_state_seed', 0)
        random_noise_seed_wk = env_config.get(
            'random_noise_seed_wk', 0)
        random_noise_seed_vk = env_config.get(
            'random_noise_seed_vk', 0)
        set_constant_reference = env_config.get(
            'set_constant_reference', False)
        constant_reference = env_config.get(
            'constant_reference', np.array([1., 0., 1., 0., 1., 0.]))
        set_custom_u_limit = env_config.get(
            'set_custom_u_limit', False)
        custom_u_high = env_config.get(
            'custom_u_high', np.array([1., 1., 1.]))
        checkForSoftLimits = env_config.get(
            'checkForSoftLimits', True)
        eval_enabled = env_config.get(
            'eval_enabled', False)
        use_casadi = env_config.get(
            'use_casadi', False)
        inertia_values = env_config.get(
            'inertia_values', initializate_inertia_values)
        set_eval_constant_reference = env_config.get(
            'set_eval_constant_reference', True)
        self.env_id = env_config.get(
            'env_id', 0)
        self.d_rand_percent = env_config.get(
            'd_rand_percent', 0.0)
        self.random_ref_mode = env_config.get(
            'random_ref_mode', 3)

        super(Quad, self).__init__()
        self.initial_inertia_values = inertia_values
        self.Ixx = inertia_values[0]
        self.Iyy = inertia_values[1]
        self.Izz = inertia_values[2]
        self.m = m
        self.g = g
        self.mg = mg
        self.d = d
        self.b = b
        self.k = k

        if eval_enabled:
            self.set_constant_reference = set_eval_constant_reference
        else:
            self.set_constant_reference = set_constant_reference
        self.constant_reference = constant_reference
        self.set_custom_u_limit = set_custom_u_limit
        self.custom_u_high = custom_u_high
        self.eval_env = eval_enabled

        self.checkForSoftLimits = checkForSoftLimits

        self.current_timestep = 0
        self.episode_timestep = 0
        self.current_time = 0
        self.initial_dynamics_state = dynamics_state
        self.dynamics_state = self.initial_dynamics_state

        self.reference_state = self.dynamics_state
        self.state = self.reference_state - self.dynamics_state

        self.db = db
        self.w_max = w_max
        self.generate_dynamics()

        self.is_linear = is_linear
        self.solver_func = linear_quad_dynamics if is_linear else \
            nonlinear_quad_dynamics

        self.current_ref_mode = self.random_ref_mode

        self.u_max = np.float32(np.array([u1_max, u2_max, u3_max]))
        self.u_min = u_min

        self.u_max_normalized = np.float32(
            np.array([1, 1, 1], dtype=np.float32))

        # self.phi_max = (u1_max/self.Ixx)*(((t_end-t_start)**2)/2)
        self.phidot_max = (u1_max/self.Ixx)*(t_end-t_start)

        # self.theta_max = (u2_max/self.Iyy)*(((t_end-t_start)**2)/2)
        self.thetadot_max = (u2_max/self.Iyy)*(t_end-t_start)

        # self.psi_max = (u3_max/self.Izz)*(((t_end-t_start)**2)/2)
        self.psidot_max = (u3_max/self.Izz)*(t_end-t_start)

        high_ = np.float32(np.array([
            np.pi,
            self.phidot_max,
            np.pi,
            self.thetadot_max,
            np.pi,
            self.psidot_max]))
        self.high = np.append(high_, high_)

        # For the error (first 6 elements), soft high values are equal with
        # the state limits since angles are limited with pi and angular
        # speed reference is always zero for this env.
        soft_ref_high_ = np.float32(np.array([
            soft_ref_diff_phi,
            soft_ref_diff_phidot,
            soft_ref_diff_theta,
            soft_ref_diff_thetadot,
            soft_ref_diff_psi,
            soft_ref_diff_psidot]))
        soft_high_ = np.float32(np.array([
            soft_phi,
            soft_phidot,
            soft_theta,
            soft_thetadot,
            soft_psi,
            soft_psidot]))
        self.soft_high = np.append(soft_ref_high_, soft_high_)

        # Quadratic Cost/Reward Matrix
        self.Q_coefficients = Q_coefficients
        self.Q = np.zeros((self.A.shape[0], self.A.shape[0]))
        self.Q[0, 0] = self.Q_coefficients[0]
        self.Q[1, 1] = self.Q_coefficients[1]
        self.Q[2, 2] = self.Q_coefficients[2]
        self.Q[3, 3] = self.Q_coefficients[3]
        self.Q[4, 4] = self.Q_coefficients[4]
        self.Q[5, 5] = self.Q_coefficients[5]

        self.R_coefficients = R_coefficients
        self.R = np.zeros((self.B.shape[1], self.B.shape[1]))
        self.R[0, 0] = self.R_coefficients[0]
        self.R[1, 1] = self.R_coefficients[1]
        self.R[2, 2] = self.R_coefficients[2]

        self.action_len = len(self.u_max)
        self.state_len = len(self.high)
        self.dynamics_len = len(self.high[0:6])
        self.internal_dynamics_len = self.dynamics_len
        self.action_space = spaces.Box(
            low=-self.u_max_normalized,
            high=self.u_max_normalized)
        self.observation_space = spaces.Box(
            low=-self.soft_high,
            high=self.soft_high)
        self.reward_range = (-1.0, 0.0)

        self.t_start = t_start
        self.t_end = t_end
        self.simulation_freq = simulation_freq
        self.simulation_timestep = 1/simulation_freq
        self.control_freq = control_freq
        self.control_timestep = 1/control_freq

        # Other initializations
        self.rnd_state = np.random.default_rng(random_state_seed)
        self.env_reset_flag = False
        self.env_hard_reset_flag = False
        self.keep_history = keep_history
        self.use_casadi = use_casadi
        self.casadi_serialized = False
        self.prev_u_normalized = np.array([0.0, 0.0, 0.0])

        # Stochastic env properties
        self.is_stochastic = is_stochastic
        self.rnd_noise_wk = np.random.default_rng(random_noise_seed_wk)
        self.rnd_noise_vk = np.random.default_rng(random_noise_seed_vk)
        self.noise_w_mean = noise_w_mean
        self.noise_w_variance = noise_w_variance
        self.noise_v_mean = noise_v_mean
        self.noise_v_variance = noise_v_variance

        # Casadi integrator initialization
        if self.use_casadi:
            self.initialize_casadi()

        # History initialization
        if self.keep_history:
            self.history = History(self.__class__.__name__)
            self.history.sol_x = np.array([])
            self.history.sol_t = np.array([])
            self.history.sol_ref = np.array([])
            self.history.sol_reward = np.array([])
            self.history.sol_actions = np.array([])
            # self.history.env_name = self.__class__.__name__
            self.history.sol_x_wo_noise = np.copy(self.history.sol_x)

    # Actions space is defined in self.action_space.
    def step(self, action):
        if action is None or np.isnan(action).any():
            reward = -1
            info = {"episode": None}
            terminated = truncated = True
            print("Quadsim: Nan received in action, resetting env.")
            self.env_reset_flag = True
            self.env_hard_reset_flag = True
            return self.state, reward, terminated, truncated, info

        # Action masking if the reference mode is not 'all'.
        if self.current_ref_mode != 3:
            action_mask = np.array([0]*3)
            action_mask[self.current_ref_mode] = 1
            action = action * action_mask

        if self.is_stochastic:
            # Randomly generating process and measurement noise
            wk = self.rnd_noise_wk.normal(
                self.noise_w_mean,
                self.noise_w_variance,
                self.dynamics_len).reshape(-1, 1)
            vk = self.rnd_noise_vk.normal(
                self.noise_v_mean,
                self.noise_v_variance,
                self.dynamics_len).reshape(-1, 1)
        else:
            wk = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1)
            vk = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1)
        # Execute one time step within the environment
        self.current_time = self.current_timestep * self.control_timestep
        time_range = (self.current_time,
                      self.current_time + self.control_timestep)

        # action_clipped is the torque commands with the
        # unit of Nm, not normalized.
        action_clipped = np.fmax(
            np.fmin(action, self.u_max_normalized),
            -self.u_max_normalized)

        if not self.is_linear:
            # minimum torque limitting for real life quadcopter matching.
            action_clipped[np.ma.masked_inside(
                action_clipped, -self.u_min, self.u_min).mask] = 0

        # Custom input limit only works for smaller
        # values than u_max_normalized.
        if self.set_custom_u_limit:
            action_clipped = np.fmax(
                np.fmin(action_clipped, self.custom_u_high),
                -self.custom_u_high)

        u1, u2, u3 = action_clipped
        # w1s, w2s, w3s, w4s = self.motor_mixing(u1, u2, u3)

        if self.use_casadi:
            if self.casadi_serialized:
                self.deserialize_casadi()
            result = self.integrator(
                x0=self.dynamics_state,
                p=casadi.vertcat(action_clipped, wk))
            next_state = result['xf'].full().flatten()
            next_time = time_range[1]
        else:
            # solve_ivp for integrating a initial value problem
            # for system of ODEs.
            # By using the max_step parameter, the simulation is ensured
            # to have minimum self.simulation_freq Hz freq.
            sol = solve_ivp(
                self.solver_func, time_range,
                self.dynamics_state,
                args=(
                    self.A, self.B, self.G,
                    self.Ixx, self.Iyy, self.Izz,
                    u1, u2, u3, wk),
                # args=(w1s, w2s, w3s, w4s),
                vectorized=True,
                max_step=self.simulation_timestep)
            if sol['status'] != 0:
                print(
                    "Integrator failed to solve. " +
                    "Nan occured. Resetting env.")
                reward = -1
                info = {"episode": None}
                terminated = truncated = True
                self.env_reset_flag = True
                self.env_hard_reset_flag = True
                return self.state, reward, terminated, truncated, info

            next_state = sol.y[:, -1]
            next_time = time_range[1]
            # Mapping the state dynamics to -pi, pi
            next_state[[0, 2, 4]] = (((
                next_state[[0, 2, 4]] + np.pi) % (2*np.pi)) - np.pi)

        # Mapping the observation values to -pi, pi
        next_state[[0, 2, 4]] = ((
            (next_state[[0, 2, 4]] + np.pi) %
            (2*np.pi)) - np.pi)

        # Adding the measurement noise to the observation
        # Flatten is used to convert 6,1 matrix to 6, vector.
        next_obs = next_state + np.dot(self.H, vk).flatten()

        # Mapping the observation values to -pi, pi
        next_obs[[0, 2, 4]] = ((
            (next_obs[[0, 2, 4]] + np.pi) %
            (2*np.pi)) - np.pi)

        current_reference_diff = self.reference_state - next_obs
        while current_reference_diff[0] > np.pi:
            current_reference_diff[0] -= 2*np.pi
        while current_reference_diff[0] < -np.pi:
            current_reference_diff[0] += 2*np.pi

        while current_reference_diff[2] > np.pi:
            current_reference_diff[2] -= 2*np.pi
        while current_reference_diff[2] < -np.pi:
            current_reference_diff[2] += 2*np.pi

        while current_reference_diff[4] > np.pi:
            current_reference_diff[4] -= 2*np.pi
        while current_reference_diff[4] < -np.pi:
            current_reference_diff[4] += 2*np.pi

        # Clipping the reference diff and next_obs with the env limits.
        current_reference_diff_clipped = np.fmax(
            np.fmin(current_reference_diff, self.soft_high[0:6]),
            -self.soft_high[0:6])
        next_obs_clipped = np.fmax(
            np.fmin(next_obs, self.soft_high[6:12]),
            -self.soft_high[6:12])

        ref_diff_normalized = np.zeros((self.internal_dynamics_len, ))
        ref_diff_normalized = (
            current_reference_diff_clipped / self.soft_high[0:6])

        action_normalized = action_clipped / self.u_max_normalized

        # action_normalized: [-1, 1]
        # prev_u_normalized: [-1, 1]
        # delta_u: ((([-1, 1] - [-1, 1])/2)**2).mean()
        # = (([-2, 2] / 2)**2).mean() = [0, 1]
        delta_u = np.fmax(np.fmin(
            (((action_normalized - self.prev_u_normalized)/2)**2).mean(),
            1), 0)

        # Since each term is normalized by its max value, to normalize the
        # entire reward/cost function, we can simply
        # normalize by dividing the sum of coefficients.
        # Reward is normalized and currently it can be in the range of [-1, 0].
        reward = -((
                ((ref_diff_normalized.T @ self.Q @ ref_diff_normalized)) +
                (((action_normalized.T @ self.R @ action_normalized))) +
                delta_u_coefficient * delta_u
            ) / (self.Q.sum() + self.R.sum() + delta_u_coefficient))

        # Sanity check for reward be in the correct range.
        if np.isnan(reward):
            print(
                "Nan receieved in reward, ending execution." +
                "(Printed for debug purposes.)")
        reward = np.fmax(np.fmin(
            reward,
            0.0), -1.0)

        terminated = False
        truncated = False
        # Checking env limits
        if self.checkEnvLimits(
                np.append(current_reference_diff, next_obs),
                self.soft_high):
            self.env_reset_flag = True
            self.env_hard_reset_flag = True
            terminated = True
            reward = -1.0

        if (self.episode_timestep + 1 >= self.t_end*self.control_freq):
            terminated = True
            self.env_reset_flag = True

            # Code for avoiding max int size.
            # If the current timestep value is too large, we reset the system.
            if self.t_end*self.control_freq >= (
                    sys.maxsize - self.current_timestep):
                self.env_hard_reset_flag = True
            if self.keep_history:
                self.t = np.reshape(self.history.sol_t, -1)

        if self.keep_history:
            self.history.sol_x_wo_noise = (np.column_stack(
                (self.history.sol_x_wo_noise, next_state))
                if self.history.sol_x_wo_noise.size else next_state)

            self.history.sol_x = (np.column_stack(
                (self.history.sol_x, next_obs_clipped)
                ) if self.history.sol_x.size else next_obs_clipped)
            self.history.sol_t = (np.column_stack((
                self.history.sol_t, next_time)
                ) if self.history.sol_t.size else np.expand_dims(
                    np.array(next_time), axis=0))

            self.history.sol_ref = (np.column_stack(
                (self.history.sol_ref, self.reference_state))
                if self.history.sol_ref.size else self.reference_state)

            self.history.sol_reward = (np.column_stack(
                (self.history.sol_reward, reward))
                if self.history.sol_reward.size else np.array([reward]))

            self.history.sol_actions = (np.column_stack(
                (self.history.sol_actions, action_clipped))
                if self.history.sol_actions.size else action_clipped)

            self.t = np.reshape(self.history.sol_t, -1)

        self.dynamics_state = next_state
        self.prev_u_normalized = action_normalized

        self.state = np.append(
            current_reference_diff_clipped, next_obs_clipped)
        self.current_timestep += 1
        self.episode_timestep += 1
        info = {"episode": None}

        return self.state, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        # Generating dynamics with domain randomization (if set)
        self.generate_dynamics()

        self.episode_timestep = 0
        if self.env_reset_flag:
            self.dynamics_state[[1, 3, 5]] = \
                self.initial_dynamics_state[[1, 3, 5]]
            self.env_reset_flag = False
            if self.env_hard_reset_flag:
                self.current_timestep = 0
                self.dynamics_state[[0, 2, 4]] = \
                    self.initial_dynamics_state[[0, 2, 4]]
                self.env_hard_reset_flag = False

        if self.set_constant_reference:
            self.reference_state = self.constant_reference
        else:
            # Generate a random reference(in radians)
            # Select a random axis (roll, pitch, yaw)
            # Random_ref_mode: 0-Roll, 1-Pitch, 2-Yaw, 3-Random
            if self.random_ref_mode == 3:
                random_axis = np.random.choice([0, 1, 2])
                self.current_ref_mode = random_axis

            random_axis = self.current_ref_mode * 2

            random_state = self.rnd_state.uniform(
                low=-random_ref_val, high=random_ref_val,
                size=self.dynamics_len)
            self.reference_state[[0, 1, 2, 3, 4, 5]] = 0.0
            # Set the generated axis to a random value.
            self.reference_state[[random_axis]] = random_state[random_axis]

        if self.current_ref_mode == 0:
            self.Q[0, 0] = self.Q_coefficients[0]
            self.Q[1, 1] = self.Q_coefficients[1]
            self.Q[2, 2] = 0.0
            self.Q[3, 3] = 0.0
            self.Q[4, 4] = 0.0
            self.Q[5, 5] = 0.0

            self.R[0, 0] = self.R_coefficients[0]
            self.R[1, 1] = 0.0
            self.R[2, 2] = 0.0

        if self.current_ref_mode == 1:
            self.Q[0, 0] = 0.0
            self.Q[1, 1] = 0.0
            self.Q[2, 2] = self.Q_coefficients[2]
            self.Q[3, 3] = self.Q_coefficients[3]
            self.Q[4, 4] = 0.0
            self.Q[5, 5] = 0.0

            self.R[0, 0] = 0.0
            self.R[1, 1] = self.R_coefficients[1]
            self.R[2, 2] = 0.0

        if self.current_ref_mode == 2:
            self.Q[0, 0] = 0.0
            self.Q[1, 1] = 0.0
            self.Q[2, 2] = 0.0
            self.Q[3, 3] = 0.0
            self.Q[4, 4] = self.Q_coefficients[4]
            self.Q[5, 5] = self.Q_coefficients[5]

            self.R[0, 0] = 0.0
            self.R[1, 1] = 0.0
            self.R[2, 2] = self.R_coefficients[2]

        if self.current_ref_mode == 3:
            self.Q[0, 0] = self.Q_coefficients[0]
            self.Q[1, 1] = self.Q_coefficients[1]
            self.Q[2, 2] = self.Q_coefficients[2]
            self.Q[3, 3] = self.Q_coefficients[3]
            self.Q[4, 4] = self.Q_coefficients[4]
            self.Q[5, 5] = self.Q_coefficients[5]

            self.R[0, 0] = self.R_coefficients[0]
            self.R[1, 1] = self.R_coefficients[1]
            self.R[2, 2] = self.R_coefficients[2]

        state_ = self.reference_state - self.dynamics_state

        if state_[0] > np.pi:
            state_[0] -= 2*np.pi
        if state_[0] < -np.pi:
            state_[0] += 2*np.pi

        if state_[2] > np.pi:
            state_[2] -= 2*np.pi
        if state_[2] < -np.pi:
            state_[2] += 2*np.pi

        if state_[4] > np.pi:
            state_[4] -= 2*np.pi
        if state_[4] < -np.pi:
            state_[4] += 2*np.pi

        # Clipping the reference diff and next_obs with the env limits.
        state_clipped = np.fmax(
            np.fmin(state_, self.soft_high[0:6]),
            -self.soft_high[0:6])
        self.dynamics_state = np.fmax(
            np.fmin(self.dynamics_state, self.soft_high[6:12]),
            -self.soft_high[6:12])

        self.state = np.append(state_, self.dynamics_state)
        info = {}
        return self.state, info

    def checkEnvLimits(self, state_arr, limits_arr):
        if np.isnan(state_arr).any():
            print("Nan occured in state_arr")
            return True
        if (np.any(np.greater_equal(state_arr,
                                    limits_arr))
            or np.any(np.less_equal(state_arr,
                                    -limits_arr))):
            return True
        return False

    def checkLimitsExceed(self, checkForSoftLimits=False):
        # Only soft limits are controlled since the reward and the physical
        # system ranges are configured according to the soft high range.
        if (np.any(np.greater_equal(self.dynamics_state,
                                    self.soft_high[0:6]))
            or np.any(np.less_equal(self.dynamics_state,
                                    -self.soft_high[0:6]))):
            return True
        return False

    def motor_mixing(self, u1, u2, u3, target_thrust=mg):
        w1_square = (target_thrust/(4*b)) + (u3 / (4*k)) + (u2 / (2*db))
        w2_square = (target_thrust/(4*b)) - (u3 / (4*k)) - (u1 / (2*db))
        w3_square = (target_thrust/(4*b)) + (u3 / (4*k)) - (u2 / (2*db))
        w4_square = (target_thrust/(4*b)) - (u3 / (4*k)) + (u1 / (2*db))
        W = np.array([w1_square, w2_square, w3_square, w4_square])
        return W

    def initialize_casadi(self):
        self.sym_x = casadi.SX.sym("x", 6)  # Differential states
        self.sym_u = casadi.SX.sym("u", 3)  # Control
        self.sym_wk = casadi.SX.sym("wk", 6)  # process noise
        if self.solver_func.__name__ == 'nonlinear_quad_dynamics':
            self.sym_xdot = casadi.vertcat(
                self.sym_x[1],
                (((self.Iyy-self.Izz) @ (self.sym_x[3]*self.sym_x[5])) +
                    (self.sym_u[0])*(u1_max)) / self.Ixx,
                self.sym_x[3],
                (((self.Izz-self.Ixx) @ (self.sym_x[1]*self.sym_x[5])) +
                    (self.sym_u[1])*(u2_max)) / self.Iyy,
                self.sym_x[5],
                (((self.Ixx-self.Iyy) @ (self.sym_x[1]*self.sym_x[3])) +
                    (self.sym_u[2])*(u3_max)) / self.Izz,
            ) + self.G @ self.sym_wk
        else:
            self.sym_xdot = casadi.vertcat(
                self.sym_x[1],
                ((self.sym_u[0])*(u1_max)) / self.Ixx,
                self.sym_x[3],
                ((self.sym_u[1])*(u2_max)) / self.Iyy,
                self.sym_x[5],
                ((self.sym_u[2])*(u3_max)) / self.Izz,
            ) + self.G @ self.sym_wk

        # Create an integrator
        ode = {
            'x': self.sym_x,
            'p': casadi.vertcat(self.sym_u, self.sym_wk),
            'ode': self.sym_xdot
            }

        opts = {
            "tf": self.control_timestep,
            'common_options': {'max_step_size': self.simulation_timestep},
            # 'verbose': True
            }

        # self.integrator = casadi.integrator(
        #     'NL_Integrator', 'cvodes', ode, opts)
        self.integrator = casadi.integrator(
            'NL_Integrator', 'rk', ode, opts)

    def serialize_casadi(self):
        self.integrator = self.integrator.serialize()
        self.sym_u = self.sym_u.serialize()
        self.sym_wk = self.sym_wk.serialize()
        self.sym_x = self.sym_x.serialize()
        self.sym_xdot = self.sym_xdot.serialize()

    def deserialize_casadi(self):
        self.integrator = self.integrator.deserialize()
        self.sym_u = self.sym_u.deserialize()
        self.sym_wk = self.sym_wk.deserialize()
        self.sym_x = self.sym_x.deserialize()
        self.sym_xdot = self.sym_xdot.deserialize()

    def reset_history_on_reset(self):
        if self.keep_history:
            self.history = History(self.__class__.__name__)
            self.history.sol_x = np.array([])
            self.history.sol_t = np.array([])
            self.history.sol_ref = np.array([])
            self.history.sol_reward = np.array([])
            self.history.sol_actions = np.array([])
            # self.history.env_name = self.__class__.__name__
            self.history.sol_x_wo_noise = np.copy(self.history.sol_x)

    def generate_dynamics(self):
        if self.eval_env:
            self.dynamics_state = self.initial_dynamics_state
        else:
            domain_rand = (
                np.random.rand(3)*(2*self.d_rand_percent)
                ) + (1-self.d_rand_percent)

            self.Ixx = self.initial_inertia_values[0] * domain_rand[0]
            self.Iyy = self.initial_inertia_values[1] * domain_rand[1]
            self.Izz = self.initial_inertia_values[2] * domain_rand[2]

        self.A = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0]],
            dtype=np.float32)

        self.B = np.array([
            [0,       0,                          0],
            [(1.0/self.Ixx)*(u1_max), 0,   0],
            [0,       0,                          0],
            [0, (1.0/self.Iyy)*(u2_max),   0],
            [0,       0,                          0],
            [0,       0,              (1.0/self.Izz)*(u3_max)]],
            dtype=np.float32)

        self.C = np.eye(6)

        self.D = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]], dtype=np.float32)

        self.G = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]],
            dtype=np.float32)

        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]],
            dtype=np.float32)


class RLWrapper(gym.Env):
    def __init__(self, env):
        super(RLWrapper, self).__init__()
        self.env = env

        self.action_space = spaces.Box(
            low=-env.u_max_normalized,
            high=env.u_max_normalized)

        self.observation_space = spaces.Box(
            low=-env.soft_high[0:6],
            high=env.soft_high[0:6])

        if "history" in dir(self.env):
            self.history = self.env.history

    def step(self, action):
        state, reward, terminated, truncated, info = self.env.step(action)
        state = state[0:6]
        return state, reward, terminated, truncated, info

    def reset(self):
        state = self.env.reset()
        state = state[0:6]
        return state


# Gym 0.21 wrapper for backwards compatibility.
class OldGymWrapper(oldgym.Env):
    def __init__(self, env):
        super(OldGymWrapper, self).__init__()
        self.env = env

        self.action_space = oldgym.spaces.Box(
            low=-self.env.u_max_normalized,
            high=self.env.u_max_normalized)
        self.observation_space = oldgym.spaces.Box(
            low=-self.env.soft_high,
            high=self.env.soft_high)

        if "history" in dir(self.env):
            self.history = self.env.history

    def step(self, action):
        state, reward, terminated, truncated, info = self.env.step(action)
        done = terminated and truncated
        return state, reward, done, info

    def reset(self):
        state, _ = self.env.reset()
        return state
