import numpy as np
import control

# LQG algorithm, (LQR with kalman filter for
# linear-quadratic-gaussian environments).
# Source: Umut Orguner EE 557 Lecture Notes 2021.
# BHD.


class LQG:
    def __init__(self, env):
        self.env = env
        self.A = self.env.A
        self.B = self.env.B
        # For the lqr-lqg notation, M and P are used for cost matrix
        # definitions instead of Q and R.
        # Q and R corresponds to the wk and vk covariance matrices.
        self.M = self.env.Q
        self.P = self.env.R

        self.Q = np.eye(self.env.dynamics_len) * (
            self.env.noise_w_variance if hasattr(
                self.env, 'noise_w_variance') else 1)
        self.R = np.eye(self.env.dynamics_len) * (
            self.env.noise_v_variance if hasattr(
                self.env, 'noise_v_variance') else 1)

        # L is the lqr gain
        # S is the solution of the algebraic riccati eqn.
        # E is the eigenvalues of the system
        self.L, self.S, self.E = control.lqr(self.A, self.B, self.M, self.P)

        self.C = self.env.C
        self.G = self.env.G
        self.H = self.env.H

        self.x_hat_k_given_kminusone = np.zeros((self.env.dynamics_len, 1))
        self.sigma_k_given_kminusone = np.eye(self.env.dynamics_len)

        self.x_hat_k_given_k = self.x_hat_k_given_kminusone
        self.sigma_k_given_k = self.sigma_k_given_kminusone

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, error_state, deterministic=True):

        # matrix to vector transformation.
        U = np.dot(self.L, self.x_hat_k_given_k)[:, 0]
        self.kalman_filter(error_state, U)
        return U

    def kalman_filter(self, y_k, u_k):
        # y_k is given (6,) and it is converted to (6,1)
        y_hat_k_given_kminusone = np.dot(self.C, self.x_hat_k_given_kminusone)

        S_k = self.C.dot(self.sigma_k_given_kminusone).dot(
            self.C.transpose()) + self.H.dot(self.R).dot(self.H.transpose())
        K_k = self.sigma_k_given_kminusone.dot(
            self.C.transpose()).dot(np.linalg.inv(S_k))

        self.x_hat_k_given_k = self.x_hat_k_given_kminusone + K_k.dot(
            (y_k.reshape(-1, 1) - y_hat_k_given_kminusone))
        self.sigma_k_given_k = self.sigma_k_given_kminusone - K_k.dot(
            S_k).dot(K_k.transpose())

        self.x_hat_k_given_kminusone = self.A.dot(
            self.x_hat_k_given_k) + self.B.dot(u_k)
        self.sigma_k_given_kminusone = self.A.dot(self.sigma_k_given_k).dot(
            self.A.transpose()) + self.G.dot(self.Q).dot(self.G.transpose())
