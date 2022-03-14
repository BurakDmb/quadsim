def experiment1():
    from quadsim.scripts.experiment1_compare_all import experiment1_compare_all
    print("---------------------------------------------------------------")
    print("Experiment 1")
    print("---------------------------------------------------------------")
    experiment1_compare_all(compare_rl_models=False,
                            plot=True, save_plot=False, loadmodel=True)


def experiment2():
    from quadsim.scripts.compare_controller_input_limits \
        import compare_controller_input_limits
    print("---------------------------------------------------------------")
    print("Experiment 2")
    print("---------------------------------------------------------------")
    compare_controller_input_limits(plot=True, save_plot=False,
                                    loadmodel=False)


def experiment4():
    from quadsim.scripts.compare_initial_conditions\
        import compare_initial_conditions
    print("---------------------------------------------------------------")
    print("Experiment 4")
    print("---------------------------------------------------------------")
    compare_initial_conditions(plot=True, save_plot=False, loadmodel=False)


def experiment5():
    from quadsim.scripts.compare_parameters import compare_parameters
    print("---------------------------------------------------------------")
    print("Experiment 5")
    print("---------------------------------------------------------------")
    compare_parameters(plot=True, save_plot=False, loadmodel=False)


if __name__ == '__main__':
    from quadsim.tests.unit_tests import unittest_main
    unittest_main()
    experiment1()
    # experiment2()
    # experiment4()
    # experiment5()
