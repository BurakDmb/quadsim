import gym
from gym import spaces
import numpy as np
from scipy.integrate import solve_ivp
from .util_history import History

Ixx = 0.0213
Iyy = 0.02217
Izz = 0.0282
m = 1.587
g = 9.81
mg = m*g
d = 0.243
b = 4.0687e-7/((2*np.pi/60)**2)
k = 8.4367e-9/((2*np.pi/60)**2)

db = d*b
w_max = 4720 * (2*np.pi/60)

A = np.array([[0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0]])

B = np.array([[0,       0,          0],
              [1.0/Ixx, 0,          0],
              [0,       0,          0],
              [0,       1.0/Iyy,    0],
              [0,       0,          0],
              [0,       0,          1.0/Izz]])

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

Q_coefficients = np.array([200, 1, 200, 1, 200, 1])
R_coefficients = np.array([10, 10, 10])

# Defining soft limits for physical life quadcopter
# maximum angular speeds, which is 2000deg/s
# Below the link for the IMU used in pixhawk 2.1 cube
# https://ardupilot.org/copter/docs/common-thecube-overview.html
# https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/
soft_phidot = np.radians(2000)
soft_thetadot = np.radians(2000)
soft_psidot = np.radians(2000)


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
    phidotdot = (np.dot((Iyy-Izz),
                        np.multiply(thetadot, psidot)) + U[0, :])/Ixx
    thetadotdot = (np.dot((Izz-Ixx),
                          np.multiply(phidot, psidot)) + U[1, :])/Iyy
    psidotdot = (np.dot((Ixx-Iyy),
                        np.multiply(phidot, thetadot)) + U[2, :])/Izz

    dxdt = np.array([x[1, :], phidotdot, x[3, :],
                    thetadotdot, x[5, :], psidotdot])

    if wk is not None:
        dxdt += np.dot(G, wk)
    return dxdt


class DeterministicQuad(gym.Env):
    """
    Linear and Deterministic Quadcopter System Dynamics
    System Input(actions): U(U1, U2, U3) torque values
    System Output(states): X = phi, phidot, theta, thetadot, psi, psidot
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, solver_func, t_start=0, t_end=5, simulation_freq=250,
                 control_freq=50,
                 dynamics_state=np.array([0, 0, 0, 0, 0, 0]),
                 keep_history=True,
                 random_state_seed=0,
                 set_constant_reference=False,
                 constant_reference=np.array([1, 0, 1, 0, 1, 0]),
                 set_custom_u_limit=False,
                 custom_u_high=np.array([1, 1, 1]),
                 checkForSoftLimits=False,
                 eval_env=False):
        super(DeterministicQuad, self).__init__()
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

        self.checkForSoftLimits = False

        self.db = db
        self.w_max = w_max
        self.A = A
        self.B = B
        self.C = C
        self.G = G
        self.H = H

        self.solver_func = solver_func

        # motor min and max speed values, max=4720 and min=3093 rpm
        # by the reference (converted into rad/s values)
        # the minimum speed value will be calculated from
        # the given mass, g and b value.
        self.w_max = w_max
        self.w_min = np.rint(np.sqrt((m * g)/(4*b)))

        u1_max = u2_max = d*b*((self.w_max**2)-(self.w_min**2))
        u3_max = k*2*((self.w_max**2)-(self.w_min**2))
        self.u_max = np.float32(np.array([u1_max, u2_max, u3_max]))

        self.phi_max = (u1_max/Ixx)*(((t_end-t_start)**2)/2)
        self.phidot_max = (u1_max/Ixx)*(t_end-t_start)

        self.theta_max = (u2_max/Iyy)*(((t_end-t_start)**2)/2)
        self.thetadot_max = (u2_max/Iyy)*(t_end-t_start)

        self.psi_max = (u3_max/Izz)*(((t_end-t_start)**2)/2)
        self.psidot_max = (u3_max/Izz)*(t_end-t_start)

        self.high = np.float32(np.array([np.pi,
                                        self.phidot_max,
                                        np.pi,
                                        self.thetadot_max,
                                        np.pi,
                                        self.psidot_max]))

        self.soft_high = np.float32(np.array([np.pi,
                                              soft_phidot,
                                              np.pi,
                                              soft_thetadot,
                                              np.pi,
                                              soft_psidot]))

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
        self.R[0, 0] = self.R_coefficients[0] / (self.u_max[0]**2)
        self.R[1, 1] = self.R_coefficients[1] / (self.u_max[1]**2)
        self.R[2, 2] = self.R_coefficients[2] / (self.u_max[2]**2)

        self.action_len = len(self.u_max)
        self.state_len = len(self.high)
        self.dynamics_len = len(self.high)
        self.action_space = spaces.Box(low=-self.u_max,
                                       high=self.u_max)
        self.observation_space = spaces.Box(low=-self.high,
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

        if self.keep_history:
            self.history = History(self.__class__.__name__)
            self.history.sol_x = np.array([])
            self.history.sol_t = np.array([])
            self.history.sol_ref = np.array([])
            self.history.sol_reward = np.array([])
            self.history.sol_actions = np.array([])

    def step(self, action):
        # Execute one time step within the environment
        self.current_time = self.current_timestep * self.control_timestep
        time_range = (self.current_time,
                      self.current_time + self.control_timestep)

        action_clipped = np.maximum(np.minimum(action, self.u_max),
                                    -self.u_max)
        if self.set_custom_u_limit:
            action_clipped = np.maximum(np.minimum(action, self.custom_u_high),
                                        -self.custom_u_high)
        u1, u2, u3 = action_clipped
        # w1s, w2s, w3s, w4s = self.motor_mixing(u1, u2, u3)

        # solve_ivp for integrating a initial value problem for system of ODEs.
        # By using the max_step parameter, the simulation is ensured
        # to have minimum 250Hz freq.
        sol = solve_ivp(self.solver_func, time_range,
                        self.dynamics_state,
                        # args=(w1s, w2s, w3s, w4s),
                        args=(u1, u2, u3),
                        vectorized=True,
                        max_step=self.simulation_timestep)

        next_state = sol.y[:, -1]
        next_time = sol.t[-1]

        # Mapping the state dynamics to -pi, pi
        next_state[[0, 2, 4]] = (((next_state[[0, 2, 4]] + np.pi) % (2*np.pi))
                                 - np.pi)
        current_reference_diff = self.reference_state - next_state

        if current_reference_diff[0] > np.pi:
            current_reference_diff[0] -= 2*np.pi
        if current_reference_diff[0] < -np.pi:
            current_reference_diff[0] += 2*np.pi

        if current_reference_diff[2] > np.pi:
            current_reference_diff[2] -= 2*np.pi
        if current_reference_diff[2] < -np.pi:
            current_reference_diff[2] += 2*np.pi

        if current_reference_diff[4] > np.pi:
            current_reference_diff[4] -= 2*np.pi
        if current_reference_diff[4] < -np.pi:
            current_reference_diff[4] += 2*np.pi

        reward = -(
            ((current_reference_diff.T @ self.Q @ current_reference_diff)) +
            (((action_clipped.T @ self.R @ action_clipped)))
            ).item() / (self.Q_coefficients.sum() + self.R_coefficients.sum())

        if self.keep_history:
            self.history.sol_x = (np.column_stack((self.history.sol_x,
                                                   next_state))
                                  if self.history.sol_x.size else next_state)

            self.history.sol_t = (np.column_stack((self.history.sol_t,
                                                   next_time))
                                  if self.history.sol_t.size else next_time)

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

        if (np.abs(self.dynamics_state[1]) +
            (self.control_timestep *
             (self.u_max[0]/self.Ixx))) > self.high[1]:
            self.env_reset_flag = True
        if (np.abs(self.dynamics_state[3]) +
            (self.control_timestep *
             (self.u_max[1]/self.Iyy))) > self.high[3]:
            self.env_reset_flag = True
        if (np.abs(self.dynamics_state[5]) +
            (self.control_timestep *
             (self.u_max[2]/self.Izz))) > self.high[5]:
            self.env_reset_flag = True

        self.state = self.reference_state - self.dynamics_state
        if self.state[0] > np.pi:
            self.state[0] -= 2*np.pi
        if self.state[0] < -np.pi:
            self.state[0] += 2*np.pi

        if self.state[2] > np.pi:
            self.state[2] -= 2*np.pi
        if self.state[2] < -np.pi:
            self.state[2] += 2*np.pi

        if self.state[4] > np.pi:
            self.state[4] -= 2*np.pi
        if self.state[4] < -np.pi:
            self.state[4] += 2*np.pi

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
        self.reference_state = self.rnd_state.uniform(low=-np.pi,
                                                      high=np.pi,
                                                      size=6)

        self.reference_state[[1, 3, 5]] = 0.0

        if self.set_constant_reference:
            self.reference_state = self.constant_reference

        self.state = self.reference_state - self.dynamics_state
        if self.state[0] > np.pi:
            self.state[0] -= 2*np.pi
        if self.state[0] < -np.pi:
            self.state[0] += 2*np.pi

        if self.state[2] > np.pi:
            self.state[2] -= 2*np.pi
        if self.state[2] < -np.pi:
            self.state[2] += 2*np.pi

        if self.state[4] > np.pi:
            self.state[4] -= 2*np.pi
        if self.state[4] < -np.pi:
            self.state[4] += 2*np.pi

        return self.state

    def checkLimitsExceed(self, checkForSoftLimits=False):
        if checkForSoftLimits:
            if (np.any(np.greater_equal(self.dynamics_state,
                                        self.soft_high))
                or np.any(np.less_equal(self.dynamics_state,
                                        -self.soft_high))):
                return True
        else:
            if (np.any(np.greater_equal(self.dynamics_state,
                                        self.high))
                or np.any(np.less_equal(self.dynamics_state,
                                        -self.high))):
                return True
        return False

    def motor_mixing(self, u1, u2, u3, target_thrust=mg):
        w1_square = (target_thrust/(4*b)) + (u3 / (4*k)) + (u2 / (2*db))
        w2_square = (target_thrust/(4*b)) - (u3 / (4*k)) - (u1 / (2*db))
        w3_square = (target_thrust/(4*b)) + (u3 / (4*k)) - (u2 / (2*db))
        w4_square = (target_thrust/(4*b)) - (u3 / (4*k)) + (u1 / (2*db))
        W = np.array([w1_square, w2_square, w3_square, w4_square])
        return W


class StochasticQuad(DeterministicQuad):
    """
    Linear and Stochastic Quadcopter System Dynamics
    System Input(actions): U(U1, U2, U3) torque values
    System Output(states): X = phi, phidot, theta, thetadot, psi, psidot
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, solver_func, t_start=0, t_end=3, simulation_freq=250,
                 control_freq=50,
                 dynamics_state=np.array([0, 0, 0, 0, 0, 0]),
                 noise_w_mean=0, noise_w_variance=0.01,
                 noise_v_mean=0, noise_v_variance=0.01,
                 keep_history=True,
                 random_state_seed=0,
                 random_noise_seed_wk=0,
                 random_noise_seed_vk=0,
                 set_constant_reference=False,
                 constant_reference=np.array([1, 0, 1, 0, 1, 0]),
                 set_custom_u_limit=False,
                 custom_u_high=np.array([1, 1, 1]),
                 checkForSoftLimits=False,
                 eval_env=False):
        super(StochasticQuad, self).\
            __init__(solver_func, t_start=t_start, t_end=t_end,
                     simulation_freq=simulation_freq,
                     control_freq=control_freq,
                     dynamics_state=dynamics_state,
                     keep_history=keep_history,
                     random_state_seed=random_state_seed,
                     set_constant_reference=set_constant_reference,
                     constant_reference=constant_reference,
                     set_custom_u_limit=set_custom_u_limit,
                     custom_u_high=custom_u_high,
                     checkForSoftLimits=checkForSoftLimits,
                     eval_env=eval_env)
        self.solver_func = solver_func
        self.rnd_noise_wk = np.random.default_rng(random_noise_seed_wk)
        self.rnd_noise_vk = np.random.default_rng(random_noise_seed_vk)
        self.noise_w_mean = noise_w_mean
        self.noise_w_variance = noise_w_variance
        self.noise_v_mean = noise_v_mean
        self.noise_v_variance = noise_v_variance

        if self.keep_history:
            self.history.env_name = self.__class__.__name__

            self.history.sol_x_wo_noise = np.copy(self.history.sol_x)

    def step(self, action):

        # Randomly generating process and measurement noise
        wk = self.rnd_noise_wk.normal(self.noise_w_mean, self.noise_w_variance,
                                      self.dynamics_len).reshape(-1, 1)
        vk = self.rnd_noise_vk.normal(self.noise_v_mean, self.noise_v_variance,
                                      self.dynamics_len).reshape(-1, 1)

        # Execute one time step within the environment
        self.current_time = self.current_timestep * self.control_timestep
        time_range = (self.current_time,
                      self.current_time + self.control_timestep)

        action_clipped = np.maximum(np.minimum(action, self.u_max),
                                    -self.u_max)

        if self.set_custom_u_limit:
            action_clipped = np.maximum(np.minimum(action, self.custom_u_high),
                                        -self.custom_u_high)
        u1, u2, u3 = action_clipped
        # w1s, w2s, w3s, w4s = self.motor_mixing(u1, u2, u3)

        # solve_ivp for integrating a initial value problem for system of ODEs.
        # By using the max_step parameter, the simulation is ensured
        # to have minimum 250Hz freq.
        sol = solve_ivp(self.solver_func, time_range,
                        self.dynamics_state,
                        args=(u1, u2, u3, wk),
                        # args=(w1s, w2s, w3s, w4s),
                        vectorized=True,
                        max_step=self.simulation_timestep)

        next_state = sol.y[:, -1]
        # Mapping the state dynamics to -pi, pi
        next_state[[0, 2, 4]] = (((next_state[[0, 2, 4]] + np.pi) % (2*np.pi))
                                 - np.pi)

        # Adding the measurement noise to the observation
        # Flatten is used to convert 6,1 matrix to 6, vector.
        next_obs = next_state + np.dot(H, vk).flatten()
        # TODO add H

        # Mapping the observation values to -pi, pi
        next_obs[[0, 2, 4]] = (((next_obs[[0, 2, 4]] + np.pi) % (2*np.pi))
                               - np.pi)
        next_time = sol.t[-1]

        current_reference_diff = self.reference_state - next_state
        if current_reference_diff[0] > np.pi:
            current_reference_diff[0] -= 2*np.pi
        if current_reference_diff[0] < -np.pi:
            current_reference_diff[0] += 2*np.pi

        if current_reference_diff[2] > np.pi:
            current_reference_diff[2] -= 2*np.pi
        if current_reference_diff[2] < -np.pi:
            current_reference_diff[2] += 2*np.pi

        if current_reference_diff[4] > np.pi:
            current_reference_diff[4] -= 2*np.pi
        if current_reference_diff[4] < -np.pi:
            current_reference_diff[4] += 2*np.pi

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

            self.history.sol_t = (np.column_stack((self.history.sol_t,
                                                   next_time))
                                  if self.history.sol_t.size else next_time)

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

        self.state = self.reference_state - self.dynamics_state

        if self.state[0] > np.pi:
            self.state[0] -= 2*np.pi
        if self.state[0] < -np.pi:
            self.state[0] += 2*np.pi

        if self.state[2] > np.pi:
            self.state[2] -= 2*np.pi
        if self.state[2] < -np.pi:
            self.state[2] += 2*np.pi

        if self.state[4] > np.pi:
            self.state[4] -= 2*np.pi
        if self.state[4] < -np.pi:
            self.state[4] += 2*np.pi

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
