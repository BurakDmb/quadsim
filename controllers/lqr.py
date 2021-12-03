import numpy as np
import control


class LQR:
    def __init__(self, env):
        self.env = env
        A = self.env.A
        B = self.env.B
        Q = self.env.Q
        R = self.env.R

        # L is the lqr gain
        # S is the solution of the algebraic riccati eqn.
        # E is the eigenvalues of the system
        self.L, self.S, self.E = control.lqr(A, B, Q, R)

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, error_state, deterministic=True):
        U = np.dot(self.L, error_state)
        return U
