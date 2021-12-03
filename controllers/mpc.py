import numpy as np


class MPC:
    def __init__(self):
        pass

    # State is the 6 dimensional vector(6,) which holds the
    # reference minus the current state information
    def predict(self, state_error):
        roll_error = state_error[0]
        pitch_error = state_error[2]
        yaw_error = state_error[4]
        U = np.array([np.sign(roll_error),
                      np.sign(pitch_error),
                      np.sign(yaw_error)])

        return U
