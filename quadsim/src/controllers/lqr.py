import numpy as np
import controlpy


class LQR:
    def __init__(self, env):
        self.env = env
        A = self.env.A
        B = self.env.B
        # For the lqr-lqg notation, M and P are used for cost matrix
        # definitions instead of Q and R.
        # Q and R corresponds to the wk and vk covariance matrices.
        Q = self.env.Q
        R = self.env.R

        # L is the lqr gain
        # S is the solution of the algebraic riccati eqn.
        # E is the eigenvalues of the system
        self.L, _, _ = controlpy.synthesis.controller_lqr(A, B, Q, R)

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, error_state, deterministic=True):
        error = error_state[0:6]
        U = np.dot(self.L, error)
        return U
