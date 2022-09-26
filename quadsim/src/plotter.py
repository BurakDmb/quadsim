
class Plotter:
    def __init__(self, controller_name):
        import matplotlib
        import matplotlib.pyplot as plt
        matplotlib.use("QT5Agg")
        self.plt = plt
        self.name = controller_name + '_'
        import os
        if not os.path.exists('results'):
            os.makedirs('results')

    def show(self):
        self.plt.show(block=True)

    def plot_only_specific_element(self,
                                   c1_env1, c1_env2,
                                   c1_env3, c1_env4,
                                   c2_env1=None, c2_env2=None,
                                   c2_env3=None, c2_env4=None,
                                   c3_env1=None, c3_env2=None,
                                   c3_env3=None, c3_env4=None,
                                   c4_env1=None, c4_env2=None,
                                   c4_env3=None, c4_env4=None,
                                   c5_env1=None, c5_env2=None,
                                   c5_env3=None, c5_env4=None,
                                   c6_env1=None, c6_env2=None,
                                   c6_env3=None, c6_env4=None,
                                   c7_env1=None, c7_env2=None,
                                   c7_env3=None, c7_env4=None,
                                   c8_env1=None, c8_env2=None,
                                   c8_env3=None, c8_env4=None,
                                   c9_env1=None, c9_env2=None,
                                   c9_env3=None, c9_env4=None,
                                   c10_env1=None, c10_env2=None,
                                   c10_env3=None, c10_env4=None,
                                   axis=0, save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)

        legend = ['Ref', 'PID']
        ylabels = ['Phi', 'Phidot', 'Theta', 'Thetadot', 'Psi', 'Psidot']
        ylabel = ylabels[axis]

        # Axis1
        # Ref and PID
        ax1.plot(c1_env1.t, c1_env1.history.sol_ref[axis, :].T)
        ax1.plot(c1_env1.t, c1_env1.history.sol_x[axis, :].T)

        if c2_env1 is not None:
            ax1.plot(c2_env1.t, c2_env1.history.sol_x[axis, :].T)
            legend.append('LQR')
        if c3_env1 is not None:
            ax1.plot(c3_env1.t, c3_env1.history.sol_x[axis, :].T)
            legend.append('LQG')
        if c4_env1 is not None:
            ax1.plot(c4_env1.t, c4_env1.history.sol_x[axis, :].T)
            legend.append('NL-MPC')
        if c5_env1 is not None:
            ax1.plot(c5_env1.t, c5_env1.history.sol_x[axis, :].T)
            legend.append('L-MPC')
        if c6_env1 is not None:
            ax1.plot(c6_env1.t, c6_env1.history.sol_x[axis, :].T)
            legend.append('DDPG')
        if c7_env1 is not None:
            ax1.plot(c7_env1.t, c7_env1.history.sol_x[axis, :].T)
            legend.append('PPO')
        if c8_env1 is not None:
            ax1.plot(c8_env1.t, c8_env1.history.sol_x[axis, :].T)
            legend.append('SAC')
        if c9_env1 is not None:
            ax1.plot(c9_env1.t, c9_env1.history.sol_x[axis, :].T)
            legend.append('TD3')
        if c10_env1 is not None:
            ax1.plot(c10_env1.t, c10_env1.history.sol_x[axis, :].T)
            legend.append('A2C')

        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel(ylabel + ' Value')
        ax1.legend(legend, shadow=True)
        ax1.set_title('Deterministic and Linear Quadcopter')

        # Axis2
        # Ref and PID
        ax2.plot(c1_env2.t, c1_env2.history.sol_ref[axis, :].T)
        ax2.plot(c1_env2.t, c1_env2.history.sol_x[axis, :].T)

        if c2_env2 is not None:
            ax2.plot(c2_env2.t, c2_env2.history.sol_x[axis, :].T)
            legend.append('LQR')
        if c3_env2 is not None:
            ax2.plot(c3_env2.t, c3_env2.history.sol_x[axis, :].T)
            legend.append('LQG')
        if c4_env2 is not None:
            ax2.plot(c4_env2.t, c4_env2.history.sol_x[axis, :].T)
            legend.append('NL-MPC')
        if c5_env2 is not None:
            ax2.plot(c5_env2.t, c5_env2.history.sol_x[axis, :].T)
            legend.append('L-MPC')
        if c6_env2 is not None:
            ax2.plot(c6_env2.t, c6_env2.history.sol_x[axis, :].T)
            legend.append('DDPG')
        if c7_env2 is not None:
            ax2.plot(c7_env2.t, c7_env2.history.sol_x[axis, :].T)
            legend.append('PPO')
        if c8_env2 is not None:
            ax2.plot(c8_env2.t, c8_env2.history.sol_x[axis, :].T)
            legend.append('SAC')
        if c9_env2 is not None:
            ax2.plot(c9_env2.t, c9_env2.history.sol_x[axis, :].T)
            legend.append('TD3')
        if c10_env2 is not None:
            ax2.plot(c10_env2.t, c10_env2.history.sol_x[axis, :].T)
            legend.append('A2C')

        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel(ylabel + ' Value')
        ax2.legend(legend, shadow=True)
        ax2.set_title('Stochastic and Linear Quadcopter (Gaussian Noise)')

        # Axis3
        # Ref and PID
        ax3.plot(c1_env3.t, c1_env3.history.sol_ref[axis, :].T)
        ax3.plot(c1_env3.t, c1_env3.history.sol_x[axis, :].T)

        if c2_env3 is not None:
            ax3.plot(c2_env3.t, c2_env3.history.sol_x[axis, :].T)
            legend.append('LQR')
        if c3_env3 is not None:
            ax3.plot(c3_env3.t, c3_env3.history.sol_x[axis, :].T)
            legend.append('LQG')
        if c4_env3 is not None:
            ax3.plot(c4_env3.t, c4_env3.history.sol_x[axis, :].T)
            legend.append('NL-MPC')
        if c5_env3 is not None:
            ax3.plot(c5_env3.t, c5_env3.history.sol_x[axis, :].T)
            legend.append('L-MPC')
        if c6_env3 is not None:
            ax3.plot(c6_env3.t, c6_env3.history.sol_x[axis, :].T)
            legend.append('DDPG')
        if c7_env3 is not None:
            ax3.plot(c7_env3.t, c7_env3.history.sol_x[axis, :].T)
            legend.append('PPO')
        if c8_env3 is not None:
            ax3.plot(c8_env3.t, c8_env3.history.sol_x[axis, :].T)
            legend.append('SAC')
        if c9_env3 is not None:
            ax3.plot(c9_env3.t, c9_env3.history.sol_x[axis, :].T)
            legend.append('TD3')
        if c10_env3 is not None:
            ax3.plot(c10_env3.t, c10_env3.history.sol_x[axis, :].T)
            legend.append('A2C')

        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel(ylabel + ' Value')
        ax3.legend(legend, shadow=True)
        ax3.set_title('Deterministic and Non Linear Quadcopter')

        # Axis4
        # Ref and PID
        ax4.plot(c1_env4.t, c1_env4.history.sol_ref[axis, :].T)
        ax4.plot(c1_env4.t, c1_env4.history.sol_x[axis, :].T)

        if c2_env4 is not None:
            ax4.plot(c2_env4.t, c2_env4.history.sol_x[axis, :].T)
            legend.append('LQR')
        if c3_env4 is not None:
            ax4.plot(c3_env4.t, c3_env4.history.sol_x[axis, :].T)
            legend.append('LQG')
        if c4_env4 is not None:
            ax4.plot(c4_env4.t, c4_env4.history.sol_x[axis, :].T)
            legend.append('NL-MPC')
        if c5_env4 is not None:
            ax4.plot(c5_env4.t, c5_env4.history.sol_x[axis, :].T)
            legend.append('L-MPC')
        if c6_env4 is not None:
            ax4.plot(c6_env4.t, c6_env4.history.sol_x[axis, :].T)
            legend.append('DDPG')
        if c7_env4 is not None:
            ax4.plot(c7_env4.t, c7_env4.history.sol_x[axis, :].T)
            legend.append('PPO')
        if c8_env4 is not None:
            ax4.plot(c8_env4.t, c8_env4.history.sol_x[axis, :].T)
            legend.append('SAC')
        if c9_env4 is not None:
            ax4.plot(c9_env4.t, c9_env4.history.sol_x[axis, :].T)
            legend.append('TD3')
        if c10_env4 is not None:
            ax4.plot(c10_env4.t, c10_env4.history.sol_x[axis, :].T)
            legend.append('A2C')

        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel(ylabel + ' Value')
        ax4.legend(legend, shadow=True)
        ax4.set_title('Stochastic and Non Linear Quadcopter (Gaussian Noise)')

        if save_plot:
            self.plt.savefig('results/' + self.name +
                             'plot_only_specific_element.png')

    def plot_compare_input_values(self,
                                  c1_env1, c1_env2,
                                  c1_env3, c1_env4,
                                  c2_env1=None, c2_env2=None,
                                  c2_env3=None, c2_env4=None,
                                  c3_env1=None, c3_env2=None,
                                  c3_env3=None, c3_env4=None,
                                  c4_env1=None, c4_env2=None,
                                  c4_env3=None, c4_env4=None,
                                  c5_env1=None, c5_env2=None,
                                  c5_env3=None, c5_env4=None,
                                  c6_env1=None, c6_env2=None,
                                  c6_env3=None, c6_env4=None,
                                  c7_env1=None, c7_env2=None,
                                  c7_env3=None, c7_env4=None,
                                  c8_env1=None, c8_env2=None,
                                  c8_env3=None, c8_env4=None,
                                  c9_env1=None, c9_env2=None,
                                  c9_env3=None, c9_env4=None,
                                  c10_env1=None, c10_env2=None,
                                  c10_env3=None, c10_env4=None,
                                  axis=0, save_plot=False):

        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)

        legend = ['PID']
        ylabels = ['U1', 'U2', 'U3']
        ylabel = ylabels[axis]

        # Axis1
        # PID
        ax1.plot(c1_env1.t, c1_env1.history.sol_actions[axis, :].T)

        if c2_env1 is not None:
            ax1.plot(c2_env1.t, c2_env1.history.sol_actions[axis, :].T)
            legend.append('LQR')
        if c3_env1 is not None:
            ax1.plot(c3_env1.t, c3_env1.history.sol_actions[axis, :].T)
            legend.append('LQG')
        if c4_env1 is not None:
            ax1.plot(c4_env1.t, c4_env1.history.sol_actions[axis, :].T)
            legend.append('NL-MPC')
        if c5_env1 is not None:
            ax1.plot(c5_env1.t, c5_env1.history.sol_actions[axis, :].T)
            legend.append('L-MPC')
        if c6_env1 is not None:
            ax1.plot(c6_env1.t, c6_env1.history.sol_actions[axis, :].T)
            legend.append('DDPG')
        if c7_env1 is not None:
            ax1.plot(c7_env1.t, c7_env1.history.sol_actions[axis, :].T)
            legend.append('PPO')
        if c8_env1 is not None:
            ax1.plot(c8_env1.t, c8_env1.history.sol_actions[axis, :].T)
            legend.append('SAC')
        if c9_env1 is not None:
            ax1.plot(c9_env1.t, c9_env1.history.sol_actions[axis, :].T)
            legend.append('TD3')
        if c10_env1 is not None:
            ax1.plot(c10_env1.t, c10_env1.history.sol_actions[axis, :].T)
            legend.append('A2C')

        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel(ylabel + ' Value')
        ax1.legend(legend, shadow=True)
        ax1.set_title('Deterministic and Linear Quadcopter')

        # Axis2
        # PID
        ax2.plot(c1_env2.t, c1_env2.history.sol_actions[axis, :].T)

        if c2_env2 is not None:
            ax2.plot(c2_env2.t, c2_env2.history.sol_actions[axis, :].T)
            legend.append('LQR')
        if c3_env2 is not None:
            ax2.plot(c3_env2.t, c3_env2.history.sol_actions[axis, :].T)
            legend.append('LQG')
        if c4_env2 is not None:
            ax2.plot(c4_env2.t, c4_env2.history.sol_actions[axis, :].T)
            legend.append('NL-MPC')
        if c5_env2 is not None:
            ax2.plot(c5_env2.t, c5_env2.history.sol_actions[axis, :].T)
            legend.append('L-MPC')
        if c6_env2 is not None:
            ax2.plot(c6_env2.t, c6_env2.history.sol_actions[axis, :].T)
            legend.append('DDPG')
        if c7_env2 is not None:
            ax2.plot(c7_env2.t, c7_env2.history.sol_actions[axis, :].T)
            legend.append('PPO')
        if c8_env2 is not None:
            ax2.plot(c8_env2.t, c8_env2.history.sol_actions[axis, :].T)
            legend.append('SAC')
        if c9_env2 is not None:
            ax2.plot(c9_env2.t, c9_env2.history.sol_actions[axis, :].T)
            legend.append('TD3')
        if c10_env2 is not None:
            ax2.plot(c10_env2.t, c10_env2.history.sol_actions[axis, :].T)
            legend.append('A2C')

        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel(ylabel + ' Value')
        ax2.legend(legend, shadow=True)
        ax2.set_title('Stochastic and Linear Quadcopter (Gaussian Noise)')

        # Axis3
        # PID
        ax3.plot(c1_env3.t, c1_env3.history.sol_actions[axis, :].T)

        if c2_env3 is not None:
            ax3.plot(c2_env3.t, c2_env3.history.sol_actions[axis, :].T)
            legend.append('LQR')
        if c3_env3 is not None:
            ax3.plot(c3_env3.t, c3_env3.history.sol_actions[axis, :].T)
            legend.append('LQG')
        if c4_env3 is not None:
            ax3.plot(c4_env3.t, c4_env3.history.sol_actions[axis, :].T)
            legend.append('NL-MPC')
        if c5_env3 is not None:
            ax3.plot(c5_env3.t, c5_env3.history.sol_actions[axis, :].T)
            legend.append('L-MPC')
        if c6_env3 is not None:
            ax3.plot(c6_env3.t, c6_env3.history.sol_actions[axis, :].T)
            legend.append('DDPG')
        if c7_env3 is not None:
            ax3.plot(c7_env3.t, c7_env3.history.sol_actions[axis, :].T)
            legend.append('PPO')
        if c8_env3 is not None:
            ax3.plot(c8_env3.t, c8_env3.history.sol_actions[axis, :].T)
            legend.append('SAC')
        if c9_env3 is not None:
            ax3.plot(c9_env3.t, c9_env3.history.sol_actions[axis, :].T)
            legend.append('TD3')
        if c10_env3 is not None:
            ax3.plot(c10_env3.t, c10_env3.history.sol_actions[axis, :].T)
            legend.append('A2C')

        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel(ylabel + ' Value')
        ax3.legend(legend, shadow=True)
        ax3.set_title('Deterministic and Non Linear Quadcopter')

        # Axis4
        # Ref and PID
        ax4.plot(c1_env4.t, c1_env4.history.sol_actions[axis, :].T)

        if c2_env4 is not None:
            ax4.plot(c2_env4.t, c2_env4.history.sol_actions[axis, :].T)
            legend.append('LQR')
        if c3_env4 is not None:
            ax4.plot(c3_env4.t, c3_env4.history.sol_actions[axis, :].T)
            legend.append('LQG')
        if c4_env4 is not None:
            ax4.plot(c4_env4.t, c4_env4.history.sol_actions[axis, :].T)
            legend.append('NL-MPC')
        if c5_env4 is not None:
            ax4.plot(c5_env4.t, c5_env4.history.sol_actions[axis, :].T)
            legend.append('L-MPC')
        if c6_env4 is not None:
            ax4.plot(c6_env4.t, c6_env4.history.sol_actions[axis, :].T)
            legend.append('DDPG')
        if c7_env4 is not None:
            ax4.plot(c7_env4.t, c7_env4.history.sol_actions[axis, :].T)
            legend.append('PPO')
        if c8_env4 is not None:
            ax4.plot(c8_env4.t, c8_env4.history.sol_actions[axis, :].T)
            legend.append('SAC')
        if c9_env4 is not None:
            ax4.plot(c9_env4.t, c9_env4.history.sol_actions[axis, :].T)
            legend.append('TD3')
        if c10_env4 is not None:
            ax4.plot(c10_env4.t, c10_env4.history.sol_actions[axis, :].T)
            legend.append('A2C')

        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel(ylabel + ' Value')
        ax4.legend(legend, shadow=True)
        ax4.set_title('Stochastic and Non Linear Quadcopter (Gaussian Noise)')

        if save_plot:
            self.plt.savefig('results/' + self.name +
                             'plot_compare_input_values.png')

    def plot_compare_parameters(self,
                                c1_env1, c1_env2, c1_env3, c1_env4,
                                c2_env1, c2_env2, c2_env3, c2_env4,
                                c3_env1, c3_env2, c3_env3, c3_env4,
                                axis=0, save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)
        ax1.plot(c1_env1.t, c1_env1.history.sol_ref[axis, :].T)
        ax1.plot(c1_env1.t, c1_env1.history.sol_x[axis, :].T)
        ax1.plot(c2_env1.t, c2_env1.history.sol_x[axis, :].T)
        ax1.plot(c3_env1.t, c3_env1.history.sol_x[axis, :].T)
        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel('Roll Angle (rad)')
        ax1.legend(['Ref', 'Prediction Horizon=10', 'Prediction Horizon=20',
                    'Prediction Horizon=100'], shadow=True)
        ax1.set_title('Deterministic Linear Quadcopter Roll Angle ' +
                      'time')

        ax2.plot(c1_env2.t, c1_env2.history.sol_ref[axis, :].T)
        ax2.plot(c1_env2.t, c1_env2.history.sol_x[axis, :].T)
        ax2.plot(c2_env2.t, c2_env2.history.sol_x[axis, :].T)
        ax2.plot(c3_env2.t, c3_env2.history.sol_x[axis, :].T)
        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel('Roll Angle (rad)')
        ax2.legend(['Ref', 'Prediction Horizon=10', 'Prediction Horizon=20',
                    'Prediction Horizon=100'], shadow=True)
        ax2.set_title('Stochastic Linear Quadcopter (Gaussian Noise)' +
                      ' - Roll Angle ' +
                      'wrt time')

        ax3.plot(c1_env3.t, c1_env3.history.sol_ref[axis, :].T)
        ax3.plot(c1_env3.t, c1_env3.history.sol_x[axis, :].T)
        ax3.plot(c2_env3.t, c2_env3.history.sol_x[axis, :].T)
        ax3.plot(c3_env3.t, c3_env3.history.sol_x[axis, :].T)
        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel('Roll Angle (rad)')
        ax3.legend(['Ref', 'Prediction Horizon=10', 'Prediction Horizon=20',
                    'Prediction Horizon=100'], shadow=True)
        ax3.set_title('Deterministic Nonlinear Quadcopter - Roll Angle' +
                      ' wrt time')

        ax4.plot(c1_env4.t, c1_env4.history.sol_ref[axis, :].T)
        ax4.plot(c1_env4.t, c1_env4.history.sol_x[axis, :].T)
        ax4.plot(c2_env4.t, c2_env4.history.sol_x[axis, :].T)
        ax4.plot(c3_env4.t, c3_env4.history.sol_x[axis, :].T)
        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel('Roll Angle (rad)')
        ax4.legend(['Ref', 'Prediction Horizon=10', 'Prediction Horizon=20',
                    'Prediction Horizon=100'], shadow=True)
        ax4.set_title('Stochastic Nonlinear Quadcopter (Gaussian Noise) -' +
                      ' Roll Angle wrt time')
        if save_plot:
            self.plt.savefig('results/' + self.name +
                             'plot_compare_parameters.png')

    def plot_reward(self, env1, env2, env3, env4, save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)
        ax1.plot(env1.t, env1.history.sol_reward[:].T)
        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel('Reward')
        ax1.legend(['Reward'], shadow=True)
        ax1.set_ylim((-1.1, 0.1))
        ax1.set_title('Deterministic Linear Quadcopter Reward Values wrt t')

        ax2.plot(env2.t, env2.history.sol_reward[:].T)
        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel('Attitude and Angular Rate Values')
        ax2.legend(['Reward'], shadow=True)
        ax2.set_ylim((-1.1, 0.1))
        ax2.set_title('Stochastic Linear Quadcopter (Gaussian Noise)' +
                      ' Reward Values wrt t')

        ax3.plot(env3.t, env3.history.sol_reward[:].T)
        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel('Attitude and Angular Rate Values')
        ax3.legend(['Reward'], shadow=True)
        ax3.set_ylim((-1.1, 0.1))
        ax3.set_title('Deterministic Nonlinear Quadcopter -' +
                      ' Reward Values wrt t')

        ax4.plot(env4.t, env4.history.sol_reward[:].T)
        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel('Attitude and Angular Rate Values')
        ax4.legend(['Reward'], shadow=True)
        ax4.set_ylim((-1.1, 0.1))
        ax4.set_title('Stochastic Nonlinear Quadcopter (Gaussian Noise) -' +
                      ' Reward Values wrt t')

        if save_plot:
            self.plt.savefig('results/' + self.name + 'plot_reward.png')

    def plot_actions(self, env1, env2, env3, env4, save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)
        ax1.plot(env1.t, env1.history.sol_actions[:].T)
        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel('Action(Torque)')
        ax1.legend(['U1', 'U2', 'U3'], shadow=True)
        ax1.set_title('Deterministic Linear Quadcopter - ' +
                      'Action(Torque) wrt t')

        ax2.plot(env2.t, env2.history.sol_actions[:].T)
        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel('Action(Torque)')
        ax2.legend(['U1', 'U2', 'U3'], shadow=True)
        ax2.set_title('Stochastic Linear Quadcopter (Gaussian Noise) -' +
                      'Action(Torque) wrt t')

        ax3.plot(env3.t, env3.history.sol_actions[:].T)
        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel('Action(Torque)')
        ax3.legend(['U1', 'U2', 'U3'], shadow=True)
        ax3.set_title('Deterministic Nonlinear Quadcopter - ' +
                      'Action(Torque) wrt t')

        ax4.plot(env4.t, env4.history.sol_actions[:].T)
        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel('Action(Torque)')
        ax4.legend(['U1', 'U2', 'U3'], shadow=True)
        ax4.set_title('Stochastic Nonlinear Quadcopter (Gaussian Noise) - ' +
                      'Action(Torque) wrt t')
        if save_plot:
            self.plt.savefig('results/' + self.name + 'plot_actions.png')

    def plot_actions_with_input_limit(self, env1, env2, env3, env4,
                                      save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)
        ax1.plot(env1.t, env1.history.sol_actions[:].T)
        ax1.hlines(y=env1.custom_u_high, xmin=env1.t[0], xmax=env1.t[-1],
                   linewidth=2, color='r')
        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel('Action(Torque)')
        ax1.legend(['U1', 'U2', 'U3'], shadow=True)
        ax1.set_title('Deterministic Linear Quadcopter - ' +
                      'Action(Torque) wrt t')

        ax2.plot(env2.t, env2.history.sol_actions[:].T)
        ax2.hlines(y=env2.custom_u_high, xmin=env2.t[0], xmax=env2.t[-1],
                   linewidth=2, color='r')
        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel('Action(Torque)')
        ax2.legend(['U1', 'U2', 'U3'], shadow=True)
        ax2.set_title('Stochastic Linear Quadcopter (Gaussian Noise) -' +
                      'Action(Torque) wrt t')

        ax3.plot(env3.t, env3.history.sol_actions[:].T)
        ax3.hlines(y=env3.custom_u_high, xmin=env3.t[0], xmax=env3.t[-1],
                   linewidth=2, color='r')
        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel('Action(Torque)')
        ax3.legend(['U1', 'U2', 'U3'], shadow=True)
        ax3.set_title('Deterministic Nonlinear Quadcopter - ' +
                      'Action(Torque) wrt t')

        ax4.plot(env4.t, env4.history.sol_actions[:].T)
        ax4.hlines(y=env4.custom_u_high, xmin=env4.t[0], xmax=env4.t[-1],
                   linewidth=2, color='r')
        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel('Action(Torque)')
        ax4.legend(['U1', 'U2', 'U3'], shadow=True)
        ax4.set_title('Stochastic Nonlinear Quadcopter (Gaussian Noise) - ' +
                      'Action(Torque) wrt t')
        if save_plot:
            self.plt.savefig('results/' + self.name + 'plot_actions.png')

    def plot_all_with_reference(self, env1, env2, env3, env4, save_plot=False):
        # Plotting the simulation recorded
        fig, ((ax1, ax2), (ax3, ax4)) = self.plt.subplots(2, 2)
        fig.set_size_inches(16, 8)
        self.plt.tight_layout(pad=5)
        ax1.plot(env1.t, env1.history.sol_ref[:].T)
        ax1.plot(env1.t, env1.history.sol_x[:].T)
        ax1.set_xlabel('Time(second)')
        ax1.set_ylabel('Attitude and Angular Rate Values')
        ax1.legend(['Ref phi(rad)', 'Ref phidot(rad/s)', 'Ref theta(rad)',
                    'Ref thetadot(rad/s)', 'Ref psi(rad)', 'Ref psidot(rad/s)',
                    'phi(rad)', 'phidot(rad/s)', 'theta(rad)',
                    'thetadot(rad/s)', 'psi(rad)', 'psidot(rad/s)'],
                   shadow=True)
        ax1.set_title('Deterministic Linear Quadcopter Attitude and ' +
                      'Angular Rate Values wrt t')

        ax2.plot(env2.t, env2.history.sol_ref[:].T)
        ax2.plot(env2.t, env2.history.sol_x[:].T)
        ax2.set_xlabel('Time(second)')
        ax2.set_ylabel('Attitude and Angular Rate Values')
        ax2.legend(['Ref phi(rad)', 'Ref phidot(rad/s)', 'Ref theta(rad)',
                    'Ref thetadot(rad/s)', 'Ref psi(rad)', 'Ref psidot(rad/s)',
                    'phi(rad)', 'phidot(rad/s)', 'theta(rad)',
                    'thetadot(rad/s)', 'psi(rad)', 'psidot(rad/s)'],
                   shadow=True)
        ax2.set_title('Stochastic Linear Quadcopter (Gaussian Noise)' +
                      ' - Attitude ' +
                      'and Angular Rate Values wrt t')

        ax3.plot(env3.t, env3.history.sol_ref[:].T)
        ax3.plot(env3.t, env3.history.sol_x[:].T)
        ax3.set_xlabel('Time(second)')
        ax3.set_ylabel('Attitude and Angular Rate Values')
        ax3.legend(['Ref phi(rad)', 'Ref phidot(rad/s)', 'Ref theta(rad)',
                    'Ref thetadot(rad/s)', 'Ref psi(rad)', 'Ref psidot(rad/s)',
                    'phi(rad)', 'phidot(rad/s)', 'theta(rad)',
                    'thetadot(rad/s)', 'psi(rad)', 'psidot(rad/s)'],
                   shadow=True)
        ax3.set_title('Deterministic Nonlinear Quadcopter - Attitude and' +
                      ' Angular Rate Values wrt t')

        ax4.plot(env4.t, env4.history.sol_ref[:].T)
        ax4.plot(env4.t, env4.history.sol_x[:].T)
        ax4.set_xlabel('Time(second)')
        ax4.set_ylabel('Attitude and Angular Rate Values')
        ax4.legend(['Ref phi(rad)', 'Ref phidot(rad/s)', 'Ref theta(rad)',
                    'Ref thetadot(rad/s)', 'Ref psi(rad)', 'Ref psidot(rad/s)',
                    'phi(rad)', 'phidot(rad/s)', 'theta(rad)',
                    'thetadot(rad/s)', 'psi(rad)', 'psidot(rad/s)'],
                   shadow=True)
        ax4.set_title('Stochastic Nonlinear Quadcopter (Gaussian Noise) -' +
                      ' Attitude and Angular Rate Values wrt t')
        if save_plot:
            self.plt.savefig('results/' + self.name +
                             'plot_all_with_reference.png')
