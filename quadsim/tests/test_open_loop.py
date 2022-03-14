import numpy as np
import sys

from quadsim.scripts.utils import test_controller


def test_all_environments_open_loop(plot=False, save_plot=False):
    class DummyController:
        def __init__(self):
            pass

        # State is the 6 dimensional vector(6,) which holds the
        # reference minus the current state information
        def predict(self, state_error, deterministic=True):
            return np.array([0, 0, 0])

    print("*** Function: ", sys._getframe().f_code.co_name, "***")
    t_end = 5
    dummyController = DummyController()
    test_controller(dummyController, t_end, plot=plot, save_plot=save_plot)


if __name__ == '__main__':
    test_all_environments_open_loop(plot=True, save_plot=False)
