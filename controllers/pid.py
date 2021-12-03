import numpy as np


class PID:
    def __init__(self, Kp, Ki, Kd, T, limMin=-np.inf, limMax=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limMin = limMin
        self.limMax = limMax
        self.T = T

        self.integrator = 0.0
        self.prevError = 0.0
        self.differentiator = 0.0
        self.prevMeasurement = 0.0

        self.errors = []
        # self.positions = []
        self.outputs = []
        self.e_prev = 0
        self.e_sum = 0
        self.pre_sat = 0
        self.u = 0

    def step(self, error):
        self.error = error
        # Anti windup, clamping
        if self.pre_sat < self.limMax and self.pre_sat > self.limMin:
            self.e_sum += (self.error*self.T)

        self.e_diff = (self.error-self.e_prev)/self.T

        proportional_term = (self.Kp*self.error)
        integral_term = (self.Ki*self.e_sum)
        derivative_term = (self.Kd * self.e_diff)

        self.u = proportional_term + integral_term + derivative_term

        self.pre_sat = self.u
        if self.u > self.limMax:
            self.u = self.limMax
        elif self.u < self.limMin:
            self.u = self.limMin

        self.outputs.append(self.u)
        self.e_prev = self.error
        self.errors.append(self.error)
        # self.positions.append(measurement)
        # print(self.u, proportional_term, integral_term, derivative_term)
        return self.u


class PID_Controller:
    def __init__(self, rollKp, rollKi, rollKd,
                 pitchKp, pitchKi, pitchKd,
                 yawKp, yawKi, yawKd, T,
                 limRoll, limPitch, limYaw):
        self.rollPID = PID(rollKp, rollKi, rollKd, T,
                           limMin=-limRoll, limMax=limRoll)
        self.pitchPID = PID(pitchKp, pitchKi, pitchKd, T,
                            limMin=-limPitch, limMax=limPitch)
        self.yawPID = PID(yawKp, yawKi, yawKd, T,
                          limMin=-limYaw, limMax=limYaw)

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, error_state, deterministic=True):
        roll_error = error_state[0]
        pitch_error = error_state[2]
        yaw_error = error_state[4]

        roll_action = self.rollPID.step(roll_error)
        pitch_action = self.pitchPID.step(pitch_error)
        yaw_action = self.yawPID.step(yaw_error)

        U = np.array([roll_action, pitch_action, yaw_action])
        return U
