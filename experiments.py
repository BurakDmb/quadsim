def experiment1():
    from test import compare_pid_lqr_nlmpc
    print("---------------------------------------------------------------")
    print("Experiment 1")
    print("---------------------------------------------------------------")
    compare_pid_lqr_nlmpc(plot=True, save_plot=False, loadmodel=False)


def experiment2():
    from test import compare_controller_input_limits
    print("---------------------------------------------------------------")
    print("Experiment 2")
    print("---------------------------------------------------------------")
    compare_controller_input_limits(plot=True, save_plot=False,
                                    loadmodel=False)


def experiment4():
    from test import compare_initial_conditions
    print("---------------------------------------------------------------")
    print("Experiment 4")
    print("---------------------------------------------------------------")
    compare_initial_conditions(plot=True, save_plot=False, loadmodel=False)


def experiment5():
    from test import compare_parameters
    print("---------------------------------------------------------------")
    print("Experiment 5")
    print("---------------------------------------------------------------")
    compare_parameters(plot=True, save_plot=False, loadmodel=False)


if __name__ == '__main__':
    from unit_tests import unittest_main
    unittest_main()
    experiment1()
    # experiment2()
    # experiment4()
    # experiment5()
