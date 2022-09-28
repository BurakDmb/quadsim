import gym
from gym import spaces
import numpy as np
from scipy.integrate import solve_ivp
from quadsim.src.envs.util_history import History
import casadi

Ixx = 0.0213  # kgm^2
Iyy = 0.02217  # kgm^2
Izz = 0.0282  # kgm^2


m = 1.587  # kg
g = 9.81  # ms^-2
mg = m*g
d = 0.450  # m
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
u_max_scale = 0.6
u_min = 0.01

A = np.array([[0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0]])

B = np.array([[0,       0,                          0],
              [(1.0/Ixx)*(u1_max*u_max_scale), 0,   0],
              [0,       0,                          0],
              [0, (1.0/Iyy)*(u2_max*u_max_scale),   0],
              [0,       0,                          0],
              [0,       0,              (1.0/Izz)*(u3_max*u_max_scale)]])

C = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])

G = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])

H = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])

# Higher the coefficient, higher the importance and the effect.
Q_coefficients = np.array([10, 0, 10, 0, 10, 0])
R_coefficients = np.array([2, 2, 2])

# Defining soft limits for physical life quadcopter
# maximum angular speeds, which is 2000deg/s
# Below the link for the IMU used in pixhawk 2.1 cube
# https://ardupilot.org/copter/docs/common-thecube-overview.html
# https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/

# Experimental soft limits.
soft_phidot = 4*np.pi
soft_thetadot = 4*np.pi
soft_psidot = 4*np.pi


# Vectorized dynamics function usage:
# y will be in shape (n,k)
# output will be in shape (n,k)
# each column corresponds to a single column in y.
def linear_quad_dynamics(t, x, u1=0, u2=0, u3=0, wk=None):
    # def linear_quad_dynamics(t, x, w1s=0, w2s=0, w3s=0, w4s=0):

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
        dxdt += np.dot(G, wk)
    return dxdt


# Vectorized dynamics function usage:
# y will be in shape (n,k)
# output will be in shape (n,k)
# each column corresponds to a single column in y.
# for quadcopter dynamics, n=6, which are
# phidot, phidotdot, thetadot, thetadotdot, psidot, psidotdot.
def nonlinear_quad_dynamics(t, x, u1=0, u2=0, u3=0, wk=None):
    # def nonlinear_quad_dynamics(t, x, w1s=0, w2s=0, w3s=0, w4s=0):

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
        np.multiply(thetadot, psidot)) + (U[0, :])*(u1_max*u_max_scale))/Ixx
    thetadotdot = (np.dot(
        (Izz-Ixx),
        np.multiply(phidot, psidot)) + (U[1, :])*(u2_max*u_max_scale))/Iyy
    psidotdot = (np.dot(
        (Ixx-Iyy),
        np.multiply(phidot, thetadot)) + (U[2, :])*(u3_max*u_max_scale))/Izz

    dxdt = np.array([x[1, :], phidotdot, x[3, :],
                    thetadotdot, x[5, :], psidotdot])

    if wk is not None:
        dxdt += np.dot(G, wk)
    return dxdt


class Quad(gym.Env):
    """
    Linear and Stochastic Quadcopter System Dynamics
    System Input(actions): U(U1, U2, U3) torque values
    System Output(states): X = phi, phidot, theta, thetadot, psi, psidot
    """
    metadata = {'render.modes': ['human']}

    def __init__(
            self, is_linear=True, is_stochastic=False,
            t_start=0, t_end=3, simulation_freq=250,
            control_freq=50,
            dynamics_state=np.array([0., 0., 0., 0., 0., 0.]),
            noise_w_mean=0, noise_w_variance=0.05,
            noise_v_mean=0, noise_v_variance=0.05,
            keep_history=True,
            random_state_seed=0,
            random_noise_seed_wk=0,
            random_noise_seed_vk=0,
            set_constant_reference=False,
            constant_reference=np.array([1., 0., 1., 0., 1., 0.]),
            set_custom_u_limit=False,
            custom_u_high=np.array([1., 1., 1.]),
            checkForSoftLimits=False,
            eval_env=False,
            use_casadi=True):

        super(Quad, self).__init__()
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.m = m
        self.g = g
        self.mg = mg
        self.d = d
        self.b = b
        self.k = k

        self.set_constant_reference = set_constant_reference
        self.constant_reference = constant_reference
        self.set_custom_u_limit = set_custom_u_limit
        self.custom_u_high = custom_u_high
        self.eval_env = eval_env

        self.checkForSoftLimits = checkForSoftLimits

        self.db = db
        self.w_max = w_max
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.H = H

        self.is_linear = is_linear
        self.solver_func = linear_quad_dynamics if is_linear else \
            nonlinear_quad_dynamics

        self.u1_max = u1_max
        self.u2_max = u2_max
        self.u3_max = u3_max
        self.u_max_scale = u_max_scale
        self.u_max = np.float32(np.array([u1_max, u2_max, u3_max]))
        self.u_min = u_min

        self.u_max_normalized = (self.u_max / self.u_max)

        self.phi_max = (u1_max/Ixx)*(((t_end-t_start)**2)/2)
        self.phidot_max = (u1_max/Ixx)*(t_end-t_start)

        self.theta_max = (u2_max/Iyy)*(((t_end-t_start)**2)/2)
        self.thetadot_max = (u2_max/Iyy)*(t_end-t_start)

        self.psi_max = (u3_max/Izz)*(((t_end-t_start)**2)/2)
        self.psidot_max = (u3_max/Izz)*(t_end-t_start)

        high_ = np.float32(np.array([
            np.pi,
            self.phidot_max,
            np.pi,
            self.thetadot_max,
            np.pi,
            self.psidot_max]))
        self.high = np.append(high_, high_)

        soft_high_ = np.float32(np.array([
            np.pi,
            soft_phidot,
            np.pi,
            soft_thetadot,
            np.pi,
            soft_psidot]))
        self.soft_high = np.append(soft_high_, soft_high_)

        # Quadratic Cost/Reward Matrix
        self.Q_coefficients = Q_coefficients
        self.Q = np.identity(A.shape[0])
        self.Q[0, 0] = self.Q_coefficients[0] / (self.soft_high[0]**2)
        self.Q[1, 1] = self.Q_coefficients[1] / (self.soft_high[1]**2)
        self.Q[2, 2] = self.Q_coefficients[2] / (self.soft_high[2]**2)
        self.Q[3, 3] = self.Q_coefficients[3] / (self.soft_high[3]**2)
        self.Q[4, 4] = self.Q_coefficients[4] / (self.soft_high[4]**2)
        self.Q[5, 5] = self.Q_coefficients[5] / (self.soft_high[5]**2)

        self.R_coefficients = R_coefficients
        self.R = np.identity(B.shape[1])
        self.R[0, 0] = self.R_coefficients[0] / (self.u_max_normalized[0]**2)
        self.R[1, 1] = self.R_coefficients[1] / (self.u_max_normalized[1]**2)
        self.R[2, 2] = self.R_coefficients[2] / (self.u_max_normalized[2]**2)

        self.action_len = len(self.u_max)
        self.state_len = len(self.high)
        self.dynamics_len = len(self.high[0:6])
        self.action_space = spaces.Box(
            low=-self.u_max_normalized,
            high=self.u_max_normalized)
        self.observation_space = spaces.Box(
            low=-self.high,
            high=self.high)

        self.t_start = t_start
        self.t_end = t_end
        self.simulation_freq = simulation_freq
        self.simulation_timestep = 1/simulation_freq
        self.control_freq = control_freq
        self.control_timestep = 1/control_freq

        self.current_timestep = 0
        self.episode_timestep = 0
        self.current_time = 0
        self.initial_dynamics_state = dynamics_state
        self.dynamics_state = self.initial_dynamics_state

        self.reference_state = self.dynamics_state
        self.state = self.reference_state - self.dynamics_state

        self.rnd_state = np.random.default_rng(random_state_seed)
        self.env_reset_flag = False
        self.keep_history = keep_history
        self.use_casadi = use_casadi
        self.casadi_serialized = False

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
            self.sym_x = casadi.SX.sym("x", 6)  # Differential states
            self.sym_u = casadi.SX.sym("u", 3)  # Control
            self.sym_wk = casadi.SX.sym("wk", 6)  # process noise
            if self.solver_func.__name__ == 'nonlinear_quad_dynamics':
                self.sym_xdot = casadi.vertcat(
                    self.sym_x[1],
                    (((Iyy-Izz) @ (self.sym_x[3]*self.sym_x[5])) +
                        (self.sym_u[0])*(u1_max*u_max_scale)) / Ixx,
                    self.sym_x[3],
                    (((Izz-Ixx) @ (self.sym_x[1]*self.sym_x[5])) +
                        (self.sym_u[1])*(u2_max*u_max_scale)) / Iyy,
                    self.sym_x[5],
                    (((Ixx-Iyy) @ (self.sym_x[1]*self.sym_x[3])) +
                        (self.sym_u[2])*(u3_max*u_max_scale)) / Izz,
                ) + G @ self.sym_wk
            else:
                self.sym_xdot = casadi.vertcat(
                    self.sym_x[1],
                    ((self.sym_u[0])*(u1_max*u_max_scale)) / Ixx,
                    self.sym_x[3],
                    ((self.sym_u[1])*(u2_max*u_max_scale)) / Iyy,
                    self.sym_x[5],
                    ((self.sym_u[2])*(u3_max*u_max_scale)) / Izz,
                ) + G @ self.sym_wk

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

            self.integrator = casadi.integrator(
                'NL_Integrator', 'rk', ode, opts)

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
            wk = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            vk = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Execute one time step within the environment
        self.current_time = self.current_timestep * self.control_timestep
        time_range = (self.current_time,
                      self.current_time + self.control_timestep)

        # action_clipped is the torque commands with the
        # unit of Nm, not normalized.
        action_clipped = np.maximum(
            np.minimum(action, self.u_max_normalized),
            -self.u_max_normalized)

        if not self.is_linear:
            # minimum torque limitting for real life quadcopter matching.
            action_clipped[np.ma.masked_inside(
                action_clipped, -self.u_min, self.u_min).mask] = 0

        # Custom input limit only works for smaller
        # values than u_max_normalized.
        if self.set_custom_u_limit:
            action_clipped = np.maximum(
                np.minimum(action_clipped, self.custom_u_high),
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
            sol = solve_ivp(self.solver_func, time_range,
                            self.dynamics_state,
                            args=(u1, u2, u3, wk),
                            # args=(w1s, w2s, w3s, w4s),
                            vectorized=True,
                            max_step=self.simulation_timestep)

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
        next_obs = next_state + np.dot(H, vk).flatten()

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

        # Since each term is normalized by its max value, to normalize the
        # entire reward/cost function, we can simply
        # normalize by dividing the sum of coefficients.
        # Reward is normalized and currently it can be in the range of [-1, 0].
        reward = -(
            ((current_reference_diff.T @ self.Q @ current_reference_diff)) +
            (((action_clipped.T @ self.R @ action_clipped)))
            ).item() / (self.Q_coefficients.sum() + self.R_coefficients.sum())

        if self.keep_history:
            self.history.sol_x_wo_noise = (np.column_stack(
                (self.history.sol_x_wo_noise, next_state))
                if self.history.sol_x_wo_noise.size else next_state)

            self.history.sol_x = (np.column_stack((self.history.sol_x,
                                                   next_obs))
                                  if self.history.sol_x.size else next_obs)

            self.history.sol_t = (
                np.column_stack((
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

        if np.abs(self.dynamics_state[1]) > (self.high[1]/2):
            self.env_reset_flag = True
        if np.abs(self.dynamics_state[3]) > (self.high[3]/2):
            self.env_reset_flag = True
        if np.abs(self.dynamics_state[5]) > (self.high[5]/2):
            self.env_reset_flag = True

        state_ = self.reference_state - next_obs

        while state_[0] > np.pi:
            state_[0] -= 2*np.pi
        while state_[0] < -np.pi:
            state_[0] += 2*np.pi

        while state_[2] > np.pi:
            state_[2] -= 2*np.pi
        while state_[2] < -np.pi:
            state_[2] += 2*np.pi

        while state_[4] > np.pi:
            state_[4] -= 2*np.pi
        while state_[4] < -np.pi:
            state_[4] += 2*np.pi

        self.state = np.append(state_, next_obs)
        self.current_timestep += 1
        self.episode_timestep += 1
        info = {"episode": None}

        done = False
        if self.checkLimitsExceed(self.checkForSoftLimits):
            self.env_reset_flag = True
            done = True

        if (self.episode_timestep >= self.t_end*self.control_freq):
            done = True
            self.env_reset_flag = True
            if self.keep_history:
                self.t = np.reshape(self.history.sol_t, -1)
        return self.state, reward, done, info

    def reset(self):
        if self.eval_env:
            self.dynamics_state = self.initial_dynamics_state

        self.episode_timestep = 0
        if self.env_reset_flag:
            self.dynamics_state[[1, 3, 5]] = \
                self.initial_dynamics_state[[1, 3, 5]]
            self.env_reset_flag = False

        # Generate a random reference(in radians)
        self.reference_state = self.rnd_state.uniform(
            low=-np.pi, high=np.pi, size=6)

        self.reference_state[[1, 3, 5]] = 0.0

        if self.set_constant_reference:
            self.reference_state = self.constant_reference

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

        self.state = np.append(state_, self.dynamics_state)
        return self.state

    def checkLimitsExceed(self, checkForSoftLimits=False):
        if checkForSoftLimits:
            if (np.any(np.greater_equal(self.dynamics_state,
                                        self.soft_high[6:12]))
                or np.any(np.less_equal(self.dynamics_state,
                                        -self.soft_high[6:12]))):
                return True
        else:
            if (np.any(np.greater_equal(self.dynamics_state,
                                        self.high[6:12]))
                or np.any(np.less_equal(self.dynamics_state,
                                        -self.high[6:12]))):
                return True
        return False

    def motor_mixing(self, u1, u2, u3, target_thrust=mg):
        w1_square = (target_thrust/(4*b)) + (u3 / (4*k)) + (u2 / (2*db))
        w2_square = (target_thrust/(4*b)) - (u3 / (4*k)) - (u1 / (2*db))
        w3_square = (target_thrust/(4*b)) + (u3 / (4*k)) - (u2 / (2*db))
        w4_square = (target_thrust/(4*b)) - (u3 / (4*k)) + (u1 / (2*db))
        W = np.array([w1_square, w2_square, w3_square, w4_square])
        return W

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


class RLWrapper(gym.Env):
    def __init__(self, env):
        super(RLWrapper, self).__init__()
        self.env = env

        self.action_space = spaces.Box(
            low=-env.u_max_normalized,
            high=env.u_max_normalized)

        self.observation_space = spaces.Box(
            low=-env.high[0:6],
            high=env.high[0:6])

        self.history = self.env.history

    def step(self, action):
        state, reward, done, info = self.env.step(action)
        state = state[0:6]
        return state, reward, done, info

    def reset(self):
        state = self.env.reset()
        state = state[0:6]
        return state
