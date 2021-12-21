import numpy as np
import control

# LQG algorithm, (LQR with kalman filter for linear-quadratic-gaussian environments).
# Source: Umut Orguner EE 557 Lecture Notes 2021. 
# BHD.

class LQG:
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

        self.C = self.env.C
        self.G = self.env.G
        self.H = self.env.H

        self.x_hat_k_given_kminusone = env.dynamics_state
        self.sigma_k_given_kminusone = 0 

        self.x_hat_k_given_k = self.x_hat_k_given_kminusone
        self.sigma_k_given_k = self.sigma_k_given_kminusone

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, error_state, deterministic=True):
        
        U = np.dot(self.L, self.x_hat_k_given_k)
        self.kalman_filter(error_state, U)
        return U

    def kalman_filter(self, y_k, u_k):
        y_hat_k_given_kminusone = np.dot(self.C, self.x_hat_k_given_kminusone)
        S_k = np.dot(np.dot(self.C,self.sigma_k_given_kminusone), self.C.transpose)
        K_k = np.dot(np.dot(self.sigma_k_given_kminusone,self.C.transpose), np.linalg.inv(S_k.inverse))

        self.x_hat_k_given_k = self.x_hat_k_given_kminusone + np.dot(K_k, (y_k - y_hat_k_given_kminusone))
        self.sigma_k_given_k = self.sigma_k_given_kminusone - np.dot(np.dot(K_k, S_k), K_k.transpose)

        self.x_hat_k_given_kminusone = np.dot(self.A, self.x_hat_k_given_k) + np.dot(self.B, u_k)
        self.sigma_k_given_kminusone = np.dot(np.dot(self.A, self.sigma_k_given_k), self.A.transpose) + np.dot(np.dot(self.G, self.Q), self.G.transpose)
