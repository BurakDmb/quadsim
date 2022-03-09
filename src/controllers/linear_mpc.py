import do_mpc

from scripts.utils import check_mpc_hsl_solver_in_path  # noqa: E402
check_mpc_hsl_solver_in_path()


class Linear_MPC:
    def __init__(self, t_end, n_horizon, c_step, s_step, env):
        # Model Paremeters
        Ixx = env.Ixx
        Iyy = env.Iyy
        Izz = env.Izz

        # Creating the Model
        model_type = 'continuous'  # either 'discrete' or 'continuous'
        model = do_mpc.model.Model(model_type)

        # Model Variables
        phi = model.set_variable(var_type='_x', var_name='phi',
                                 shape=(1, 1))
        phidot = model.set_variable(var_type='_x', var_name='phidot',
                                    shape=(1, 1))
        theta = model.set_variable(var_type='_x', var_name='theta',
                                   shape=(1, 1))
        thetadot = model.set_variable(var_type='_x', var_name='thetadot',
                                      shape=(1, 1))
        psi = model.set_variable(var_type='_x', var_name='psi',
                                 shape=(1, 1))
        psidot = model.set_variable(var_type='_x', var_name='psidot',
                                    shape=(1, 1))

        tau_phi = model.set_variable(var_type='_u', var_name='tau_phi')
        tau_theta = model.set_variable(var_type='_u', var_name='tau_theta')
        tau_psi = model.set_variable(var_type='_u', var_name='tau_psi')

        # Right-hand-side equation
        model.set_rhs('phi', phidot)
        model.set_rhs('phidot',  tau_phi/Ixx)
        model.set_rhs('theta', thetadot)
        model.set_rhs('thetadot', tau_theta/Iyy)
        model.set_rhs('psi', psidot)
        model.set_rhs('psidot', tau_psi/Izz)

        # Model Setup
        model.setup()

        # Configuring the MPC controller
        self.mpc = do_mpc.controller.MPC(model)

        # Optimizer Parameters

        setup_mpc = {
            'n_horizon': n_horizon,
            't_step': c_step,
            'n_robust': 1,
            'store_full_solution': True,
            # 'nlpsol_opts' : {'ipopt.linear_solver': 'MA27'},
            'nlpsol_opts': {'ipopt.print_level': 0, 'ipopt.sb': 'yes',
                            'print_time': 0, 'ipopt.linear_solver': 'MA27'},
        }
        self.mpc.set_param(**setup_mpc)

        # Objective Function
        u1_max = env.u_max[0]
        u2_max = env.u_max[1]
        u3_max = env.u_max[2]

        phi_max = env.high[0]
        theta_max = env.high[2]
        psi_max = env.high[4]
        phidot_max = env.high[1]
        thetadot_max = env.high[3]
        psidot_max = env.high[5]

        mterm = (env.Q[0, 0]*(phi**2) +
                 env.Q[1, 1]*(theta**2) +
                 env.Q[2, 2]*(psi**2) +
                 env.Q[3, 3]*(phidot**2) +
                 env.Q[4, 4]*(theta**2) +
                 env.Q[5, 5]*(psidot**2))

        lterm = (env.Q[0, 0]*(phi**2) +
                 env.Q[1, 1]*(theta**2) +
                 env.Q[2, 2]*(psi**2) +
                 env.Q[3, 3]*(phidot**2) +
                 env.Q[4, 4]*(theta**2) +
                 env.Q[5, 5]*(psidot**2) +
                 env.R[0, 0]*(tau_phi**2) +
                 env.R[1, 1]*(tau_theta**2) +
                 env.R[2, 2]*(tau_psi**2)
                 )

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        # Penality for the Control Inputs
        self.mpc.set_rterm(
            tau_phi=0,
            tau_theta=0,
            tau_psi=0
        )

        # Lower bounds on states:
        self.mpc.bounds['lower', '_x', 'phi'] = -2*phi_max
        self.mpc.bounds['lower', '_x', 'phidot'] = -phidot_max
        self.mpc.bounds['lower', '_x', 'theta'] = -2*theta_max
        self.mpc.bounds['lower', '_x', 'thetadot'] = -thetadot_max
        self.mpc.bounds['lower', '_x', 'psi'] = -2*psi_max
        self.mpc.bounds['lower', '_x', 'psidot'] = -psidot_max

        # Upper bounds on states
        self.mpc.bounds['upper', '_x', 'phi'] = 2*phi_max
        self.mpc.bounds['upper', '_x', 'phidot'] = phidot_max
        self.mpc.bounds['upper', '_x', 'theta'] = 2*theta_max
        self.mpc.bounds['upper', '_x', 'thetadot'] = thetadot_max
        self.mpc.bounds['upper', '_x', 'psi'] = 2*psi_max
        self.mpc.bounds['upper', '_x', 'psidot'] = psidot_max

        # Lower bounds on inputs:
        self.mpc.bounds['lower', '_u', 'tau_phi'] = -u1_max
        self.mpc.bounds['lower', '_u', 'tau_theta'] = -u2_max
        self.mpc.bounds['lower', '_u', 'tau_psi'] = -u3_max
        # Upper bounds on inputs:
        self.mpc.bounds['upper', '_u', 'tau_phi'] = u1_max
        self.mpc.bounds['upper', '_u', 'tau_theta'] = u2_max
        self.mpc.bounds['upper', '_u', 'tau_psi'] = u3_max

        # Setup
        self.mpc.setup()

        # Configuring the Simulator
        self.simulator = do_mpc.simulator.Simulator(model)

        # Simulator parameters
        # Instead of supplying a dict with the splat operator (**),
        # as with the optimizer.set_param(),
        # we can also use keywords (and call the
        # method multiple times, if necessary):
        self.simulator.set_param(t_step=s_step)

        # Setup
        self.simulator.setup()

        # Creating the control loop
        self.x0 = env.dynamics_state.reshape(-1, 1)

        # Use the x0 property to set the initial state

        self.simulator.x0 = self.x0
        self.mpc.x0 = self.x0

        # Set the initial guess of the MPC optimization problem
        self.mpc.set_initial_guess()

        self.simulator.reset_history()
        self.simulator.x0 = self.x0
        self.mpc.reset_history()

    def predict(self, error, deterministic=True):
        u0 = -self.mpc.make_step(error)
        self.x0 = self.simulator.make_step(u0)
        return u0.reshape((3,))
