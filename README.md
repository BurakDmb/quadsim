# Quadcopter rotational system dynamics formulation, both nonlinear and linear representations


This study focuses on designing and developing controllers to control angular rates of a quadcopter. Therefore, only the rotational dynamics of the quadcopter are taken into consideration.
Below, you can see the assumptions made before a mathematical model:

* The rotational motion of the quadcopter is independent of its translational motion.
* The center of gravity coincides with the origin of the body-fixed frame.
* The structure of the quadcopter is rigid and symmetrical with the four arms coinciding with the body x- and y-axes.
* Drag and thrust forces are proportional to the square of the propellers speed.
* The propellers are rigid.

## Mathematical Formulation, Definitions, Dynamics

### Coordinate Frames

Coordinate frames are needed to describe the motions of the quadcopter before the quadcopter is mathematically modeled. Earth Fixed Frame W and Body Fixed Frame B is defined like this:

<p align="center">
  <img src="images/CoordinateFrames.png" alt="Coordinate Frame">
</p>

The earth-fixed frame is taken as the reference frame using the NED (North East Down) convention where the x-axis of the frame is pointed to the north.

The orientation of the quadcopter, known as attitude is expressed in the body-fixed frame by euler angles phi, theta, psi which corresponds to the roll, pitch and yaw angles.

In order to relate the orientation of the quadcopter in the earth-fixed frame, a rotation matrix is defined. Below, the rotation matrix R is defined, it converts from the body-fixed frame to the earth-fixed frame:

```math
\bold{R} =
\begin{bmatrix}
   C_{\psi}C_{\theta} & C_{\psi}S_{\theta}S_{\phi} - S_{\psi}C_{\phi} & C_{\psi}S_{\theta}C_{\phi} + S_{\psi}S_{\phi} \\
   
   S_{\psi}C_{\theta} & S_{\psi}S_{\theta}S_{\phi} + C_{\psi}C_{\phi} & S_{\psi}S_{\theta}C_{\phi} - C_{\psi}S_{\phi} \\
   -S_{\theta} & C_{\theta}S_{\phi} & C_{\theta}C_{\phi} \\
\end{bmatrix}
```

With the rotation matrix, you can calculate the earth-fixed position of any given position.

```math
\bold{v^{'}} = \bold{Rv}\\[10pt]
\bold{v} = \begin{bmatrix}
x\\
y\\
z
\end{bmatrix}
```

### Quadcopter body frame formulation

A typical quadcopter is driven by four rotors on the four arms of the frame. The position of the arms according to the body frame can differ. Generally, the most used configurations are "X" and "+" which represents the shape of x and plus shape.

In this study, the "X" shape frame were used and below, you can see the numerated rotors(from 1 to 4), rotation of the numerated rotors(clockwise or counter cw) and the body frame.

<p align="center">
  <img src="images/RotorDirections.png" alt="Rotor Directions">
</p>

<p align="center">
  <img src="images/RotationalDirections.png" alt="Rotational Directions">
</p>

### From the rotational equations of motion, quadcopter dynamics can be written out as

Let us define the inputs as:

```math
U_{1} = db(w_{4}^2-w_{2}^2) \\
U_{2} = db(w_{1}^2-w_{3}^2) \\
U_{3} = k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2)
```

Where,

```math
b \text{ - thrust coefficient} \\
k \text{ - aerodrag coefficient} \\
d \text{ - moment arm} \\
w \text{ - motor speed}
```

We can now define the nonlinear system dynamics as:

```math
\ddot{\phi} =  \frac{(I_{yy}- I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}} + \frac{U_{1}}{I_{xx}} \\
\ddot{\theta} =  \frac{(I_{zz}- I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}} + \frac{U_{2}}{I_{yy}} \\
\ddot{\psi} =  \frac{(I_{xx}- I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}} + \frac{U_{3}}{I_{zz}} \\

```

For both linear and nonlinear systems, below you can find the state and input definitions:

```math
x = 
\begin{bmatrix}
   \phi \\
   \dot{\phi} \\
   \theta \\
   \dot{\theta} \\
   \psi \\
   \dot{\psi} \\
\end{bmatrix}
, \dot{x} = 
\begin{bmatrix}
   \dot{\phi} \\
   \ddot{\phi} \\
   \dot{\theta} \\
   \ddot{\theta} \\
   \dot{\psi} \\
   \ddot{\psi} \\
\end{bmatrix}
\\[10pt]
u =
\begin{bmatrix}
   U1 \\
   U2 \\
   U3 \\
\end{bmatrix}
=
\begin{bmatrix}
   db(w_{4}^2-w_{2}^2) \\
   db(w_{1}^2-w_{3}^2) \\
   k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2) \\
\end{bmatrix}
```

### State space(linear) representation of the quadcopter system

A LTI state space model is written below for model analysis and numerical calculations.

```math
\dot{x}= Ax + Bu\\
y = Cx + Du
```

The system was linearized at the hover position and system can fully observable

```math
A = 
\begin{bmatrix}
   0 & 1 & 0 & 0 & 0 & 0 \\
   0 & 0 & 0 & 0 & 0 & 0 \\
   0 & 0 & 0 & 1 & 0 & 0 \\
   0 & 0 & 0 & 0 & 0 & 0 \\
   0 & 0 & 0 & 0 & 0 & 1 \\
   0 & 0 & 0 & 0 & 0 & 0 
\end{bmatrix}
,B =
\begin{bmatrix}
   0 & 0 & 0 \\
   \frac{1}{I_{xx}} & 0 & 0 \\
   0 & 0 & 0 \\
   0 & \frac{1}{I_{yy}} & 0 \\
   0 & 0 & 0 \\
   0 & 0 & \frac{1}{I_{zz}}  
\end{bmatrix}
```

```math
C = 
\begin{bmatrix}
   1 & 0 & 0 & 0 & 0 & 0 \\
   0 & 1 & 0 & 0 & 0 & 0 \\
   0 & 0 & 1 & 0 & 0 & 0 \\
   0 & 0 & 0 & 1 & 0 & 0 \\
   0 & 0 & 0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 0 & 0 & 1 
\end{bmatrix}
,D = 
\begin{bmatrix}
   0 & 0 & 0 \\
   0 & 0 & 0 \\
   0 & 0 & 0 \\
   0 & 0 & 0 \\
   0 & 0 & 0 \\
   0 & 0 & 0 
\end{bmatrix}
```

Where the state vector x and the input vector u is defined like this:

```math
x = 
\begin{bmatrix}
   \phi \\
   \dot{\phi} \\
   \theta \\
   \dot{\theta} \\
   \psi \\
   \dot{\psi} \\
\end{bmatrix}
, \dot{x} = 
\begin{bmatrix}
   \dot{\phi} \\
   \ddot{\phi} \\
   \dot{\theta} \\
   \ddot{\theta} \\
   \dot{\psi} \\
   \ddot{\psi} \\
\end{bmatrix}
\\[10pt]
u =
\begin{bmatrix}
   U1 \\
   U2 \\
   U3 \\
\end{bmatrix}
=
\begin{bmatrix}
   db(w_{4}^2-w_{2}^2) \\
   db(w_{1}^2-w_{3}^2) \\
   k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2) \\
\end{bmatrix}
```

### Nonlinear dynamics

```math
\dot{x} = f(x,u)\\
y = g(x,u)
\\[30pt]
\dot{x} = f(x,u) =
\begin{bmatrix}
   \dot{\phi} \\[10pt]
   \frac{(I_{yy}- I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}} + \frac{U_{1}}{I_{xx}} \\[10pt]
   \dot{\theta} \\[10pt]
   \frac{(I_{zz}- I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}} + \frac{U_{2}}{I_{yy}} \\[10pt]
   \dot{\psi} \\[10pt]
   \frac{(I_{xx}- I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}} + \frac{U_{3}}{I_{zz}} \\[10pt]
\end{bmatrix}
\\[20pt]
y = g(x,u) = x = 
\begin{bmatrix}
   \phi \\
   \dot{\phi} \\
   \theta \\
   \dot{\theta} \\
   \psi \\
   \dot{\psi} \\
\end{bmatrix}
```

## Simulation

### Solving the ODEs

* To solve the ordinary differential equations, the scipy python package was selected.

* In the scipy library, solve_ivp function with RK45 method(Explicit Runge-Kutta method of order 5(4)) has been used.

* Within the solver parameters, this simulation aimed to have 250Hz simulation step frequency.

Ref: <https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html#scipy-integrate-solve-ivp>

### Gym Environment

To ease up the usage of the simulation for RL and other agents/algorithms, the simulation has been wrapped up with OpenAI Gym Framework.

#### Environment States and Actions

##### States

   With the given system dynamics, the aim is to track reference angle signals(roll, pitch and yaw angles) in the best way. Therefore, we augment the state with extra 6 dimensional information which tells what reference signals must the controller follow.

   To summarize, the augmented state is defined like this:

   ```math
   x = 
   \begin{bmatrix}
      \phi_{err} \\
      \dot{\phi}_{err} \\
      \theta_{err} \\
      \dot{\theta}_{err} \\
      \psi_{err} \\
      \dot{\psi}_{err} \\
   \end{bmatrix}
   ```

Note that error variables are the difference between the reference state and current state.

The angle states are also mapped between [-pi, pi).

##### Actions

   The actions are defined as the original definition, the action is a vector with size `(3,)` and it tells the torque/moment vector acting on the quadcopter in the body-fixed frame.

   ```math
   U =
   \begin{bmatrix}
         u_{1} \\
         u_{2} \\
         u_{3} \\
   \end{bmatrix}
   =
   \begin{bmatrix}
         \tau_{\phi} \\
         \tau_{\theta} \\
         \tau_{\psi} \\
   \end{bmatrix}
   ```

   The environment will automatically calculate the propeller speeds with its motor mixing algorithm. So the agent only needs to give the environment the torque vector.

#### Environment Limits(min and max)

   After defining states and actions, the hard limits of these needs to be calculated. These limits needs to be defined in the Gym environment class.

   From the datasheet of the EMAX 650kv dc motors, the maximum speed rating of a dc motor is 4720rpm.
   To calculate the minimum speed rating, we need to calculate the minimum thrust for a quadcopter to hover its weight.

   ```math
   m=1.587\\
   g=9.81\\
   F=m*g=15.57N
   ```

   ```math
   \text{Thrust} = 
   b\sum_{i=1}^{4}w_{i}^{2}
   ```

   Therefore, to achieve the minimum trust, the motor speeds needs to be

   ```math
   w_{min} = w_{i} = \sqrt{\frac{15.57}{4 * 4.0687 * 10^{-7}}} \approx 3093 \text{ rpm}\\[10pt]

   w_{max} = 4720 \text{ rpm (from motor datasheet)}
   ```

   ```math
   rad/s = \frac{RPM*2\pi}{60}\\[10pt]
   w_{max} = 494.28 \text{ rad/s}\\[10pt]
   w_{min} = 323.90 \text{ rad/s}

   ```

   From the motor min-max speed values, we can calculate the U min and max values:

   ```math
   U_{max} = 
   \begin{bmatrix}
      U_{1}\\
      U_{2}\\
      U_{3}\\
   \end{bmatrix}
   =
   \begin{bmatrix}
      db((w^2_{max})-(w^2_{min})) \\
      db((w^2_{max})-(w^2_{min})) \\
      2k((w^2_{max})-(w^2_{min})) \\
   \end{bmatrix}\\[10pt]

   U_{min} = -U_{max}
   ```

   After that, we can calculate maximum state values when the motor speeds are constant by calculating the definite integral over this period. Please note that rotational values are already bounded, so we dont need to calculate the maximum value of them.

   ```math

   \phi_{max} = \pi \quad \text{ rad}\\[10pt]
   \dot{\phi}_{max}= \frac{U_{1max}}{Ixx}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt]
   \theta_{max}= \pi \quad \text{ rad}\\[10pt]
   \dot{\theta}_{max} = \frac{U_{2max}}{Iyy}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt]
   \psi_{max} = \pi  \quad \text{ rad}\\[10pt]
   \dot{\psi}_{max} =  \frac{U_{3max}}{Izz}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt]

   ```

#### Control Algorithm and Simulation Frequency

   For this simulation, a simulation frequency of __250Hz__ and controller(control algorithm) frequency of __50Hz__ have been selected and used.

   From the openai gym perspective, the action selected from the algorithm will be constant for every 0.02(1/50) seconds.

#### Rewards/Costs

   For the reward function, the quadratic cost of error(reference-current) has been used.

   ```math
   \overline{x} = x_{ref}-x_{current} \\[10pt]
   
   cost = \overline{x}^{T}Q\overline{x} + u^{T}Ru \\[10pt]
   
   Q = identity(6,6) / \text{max state values} \\[10pt]
   
   R = identity(3,3) / \text{max action values} \\[10pt]

   \text{reward} = -\text{cost} \\
   ```

   Where Q and R are reward/cost matrices, which are assumed to be normalized identity matrices of shape (6,6) and (3,3) respectively.

   Also, the shape of the current and reference states are (6,1) vectors.

#### Reset Function

   In each reset, reference difference of phi, theta and psi states are randomly generated in range [-pi,pi).
   Because of the current angles are mapped into the range [-pi, pi), the environment also needs to handle the shortest turn angle for some edge cases.

#### Done Condition

   Time is the only designed end condition for this simulation. This is because the simulation states and inputs are bounded for given time, so there is no need for ending the simulation earlier than planned. Parameter `t_start` and `t_end` defines how long each episode will be.

   Because of the implemented systems are time invariant, this time information only defines the episode length of the simulator.

#### Environment Parameters Default Values and Meanings

   The signatures of the environments are given as follow:

   ```python
   DeterministicQuad(solver_func, t_start=0, t_end=5, simulation_freq=250,
                 control_freq=50,
                 dynamics_state=np.array([0, 0, 0, 0, 0, 0]),
                 keep_history=True,
                 random_state_seed=0,
                 set_constant_reference=False,
                 constant_reference=1,
                 set_custom_u_limit=False,
                 custom_u_high=np.array([1, 1, 1])):
   ```

   ```python
   StochasticQuad(solver_func, t_start=0, t_end=3, simulation_freq=250,
                 control_freq=50,
                 dynamics_state=np.array([0, 0, 0, 0, 0, 0]),
                 noise_w_mean=0, noise_w_variance=0.01,
                 noise_v_mean=0, noise_v_variance=0.01,
                 keep_history=True,
                 random_state_seed=0,
                 random_noise_seed_wk=0,
                 random_noise_seed_vk=0,
                 set_constant_reference=False,
                 constant_reference=1,
                 set_custom_u_limit=False,
                 custom_u_high=np.array([1, 1, 1])):
   ```

* __`solver_func`__ is the solver function, which can be `linear_quad_dynamics` or `nonlinear_quad_dynamics` functions. These functions are also defined in the `quad.py` file.

* __`t_start` `t_end`__ are the simulation start and end time. For now, it only affects the episode length because of these systems are time invariant.

* __`simulation_freq`__ is the simulation frequency of the given environment, this parameter only effects the integrator/solver of the environment, not affects the control frequency. It needs to be greater than the `control_freq` parameter.

* __`control_freq`__ is the controller frequency rate which the controller makes actions in each (1/control_freq) timestep. The rate of the gym step() function is only affected by this parameter.

* __`dynamics_state`__ is the initial state of the environment.

* __`noise_w_mean`__ and __`noise_w_variance`__ is the gaussian process noise mean and variance parameter.

* __`noise_v_mean`__ and __`noise_v_variance`__ is the gaussian measurement noise mean and variance parameter.

* __`keep_history`__ parameter enables to keep the state history for this environment from initializing the object to end of the all episodes. All of the history is saved in the object `env.history` if this parameter is true.

* __`random_state_seed`__, __`random_noise_seed_wk`__, __`random_noise_seed_vk`__ is the seed parameter for the random generators. By default these parameters are set to zero.

* __`set_constant_reference`__ and __`constant_reference`__ parameters enables the use of constant reference for the environments. The first parameter is the boolean parameter to enable this functionality and the second parameter is the value of this feature.

* __`set_custom_u_limit`__ and __`custom_u_high`__ parameters enables the use of custom input(U) limits for the system. The first parameter is the boolean parameter to enable this functionality and the second parameter is the value of this feature.

### Quadcopter Simulation Parameters

```math
\def\arraystretch{1.5}
   \begin{array}{c:c}
   \text{Mass, m} & 1.587 \quad kg \\ 
   \text{Moment arm, d} & 0.243 \quad m \\
   \text{Thrust coefficient, b} & 3.7102 * 10^{-5} \quad Ns^{2}\\
   \text{Drag coefficient, k} & 7.6933 * 10^{-7} \quad Nms^{2}\\
   \text{Moment of inertia about x-axis, }I_{xx} & 0.0213 \quad kgm^{2} \\
   \text{Moment of inertia about y-axis, }I_{yy} & 0.02217 \quad kgm^{2} \\
   \text{Moment of inertia about z-axis, }I_{zz} & 0.0282 \quad kgm^{2} \\
\end{array}
```

## Controllers

### PID Control Algorithm

A proportional-integral-derivative(PID) controller is a control loop mechanism for controlling the error signal(difference between the desired setpoint and measured variable) by applying proportional, integral, derivative terms to the system input.

The mathematical form of the overall control function is:

```math
u(t) = K_{p}e(t) + K_{i} \int_{0}^{t}e(\tau)d\tau + K_{d} \frac{de(t)}{dt}\\[10pt]
\text{Where }K_{p}, K_{i}\text{ and }K_{d}\\[10pt]
\text{ are all non-negative proportional, integral and derivative terms respectively.}
```

Since PID control algorithm only works for single-input, single-output systems, 3 parallel PID controllers are needed for roll, pitch, yaw angle control.

Matlab-Simulink has been used for auto tune functionality.

Below, the tuned parameters are listed:

```math
\def\arraystretch{1.5}
   \begin{array}{c:c}
   \text{Roll, } K_{p}  & 4.72531916175911 \\
   \text{Roll, } K_{i}  & 3.73086282471387 \\
   \text{Roll, } K_{d}  & 1.49621161575424 \\[10pt]
   \text{Pitch, } K_{p}  & 3.63871317561002 \\
   \text{Pitch, } K_{i}  & 2.14232438611662 \\
   \text{Pitch, } K_{d}  & 1.54507805402352 \\[10pt]
   \text{Yaw, } K_{p}  & 4.6284037687056 \\
   \text{Yaw, } K_{i}  & 2.72501342753779 \\
   \text{Yaw, } K_{d}  & 1.96532255856848 \\
\end{array}
```

### MPC Algorithm

Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. Model predictive controllers rely on dynamic models of the process, most often linear empirical models obtained by system identification.

The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots in account. This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot and then optimizing again, repeatedly, thus differing from Linear-Quadratic Regulator (LQR).

Also MPC has the ability to anticipate future events and can take control actions accordingly. PID controllers do not have this predictive ability.

Below, the tuned parameters are listed:

```math
N_{p} = 50,\\[10pt]
t_{step} = 0.02,\\[10pt]
n_{robust} = 1,\\[10pt]
t_{end} = 10,\\[10pt]
linear solver = MA27,\\[10pt]
```

### Infinite Horizon Discrete Linear Quadratic Regulator (LQR)

For Linear Quadratic Gaussian Systems, the dynamics are described as below:

```math
x_{k+1}= Ax_{k} + Bu_{k} + Gw_{k}\\
y_{k} = Cx_{k} + Hv_{k}\\
u_{k} = g_{k}(y_{0:k})\\[10pt]

x_{0} \sim \mathcal{N}(x_{0}; \overline{x_{0}}, \Sigma_{0})\\
w_{k} \sim \mathcal{N}(w_{k}; 0, \Sigma_{w})\\
v_{k} \sim \mathcal{N}(v_{k}; 0, \Sigma_{v})\\[10pt]
J=\sum_{k=0}^{N}c_{k}(x_{k},u_{k})\\[10pt]
\text{where: }\\[10pt]
c_{k}(x_{k},u_{k}) \triangleq x_{k}^{T}Q_{k}x_{k} + u_{k}^{T}R_{k}u_{k}, \quad 0\leq k\leq N-1 \\[10pt]
c_{N}(x_{N},u_{N}) = c_{N}(x_{N}) = x_{N}^{T}Q_{N}x_{N}\\[10pt]

```

In the complete information case, C is the identity matrix, D is zero and y will be:

```math
y_{k}=x_{k} \quad \forall k
```

For LQR control, Sigma w and Sigma v are also needs to be 0, which means that systems are deterministic.

Also, the matrix Q needs to be positive semi definite(>=0) and the matrix R needs to be positive definite (>0).

#### Regulation task

In the Linear Quadratic Regulator Control (LQR), we can define the optimal feedback policy with given cost function J to regulate the states and inputs to zero.

We can calculate the optimal policy for a finite horizon N with the equations below:

```math
\overline{W} = A^{T}(\overline{W} - \overline{W} B(B^{T}\overline{W}B + R)^{-1} A + Q \\[10pt]
\overline{L} = (B^{T}\overline{W}B + R)^{-1} B^{T}\overline{W}A \\[10pt]

\bold{
   u_{k}^{*} = -\overline{L}x_{k}
} \\[10pt]
```

The dimensions of these matrix as follows:

```math
A = (6,6)\\
B = (6,3)\\[10pt]
Q = (6,6)\\
R = (3,3)\\[10pt]
W = (6,6)\\
L = (3,6)\\
```

Below, you can see the pseudocode of the LQR algorithm:

```python
def initLQR():
   Define A, B # from the system dynamics
   Define Q, R # from defined cost function
   L = SolveDARE(A, B, Q, R)

# The simulation starts, we can return the jth state with current state
def getLQRActionAtCurrentState(xcurrent):
   uk = -Lk*xcurrent
   return uk
```

#### Tracking task

Until now, the objective was to regulate the state into zero. For the tracking task, we can augment the state as reference minus current state with only one difference, the sign of u. The other formulations are same with the previous formulations.

```math
\bold{\overline{x_{k}} = x_{ref} - x_{k}}\\[10pt]
\bold{u_{k}^{*} = \overline{L}\overline{x_{k}}}
```

Below, you can see the pseudocode for LQR with tracking task:

```python
# calculates the LQR gain, same with the regulation problem
def initLQR():
   Define A, B # from the system dynamics
   Define Q, R # from defined cost function
   L = SolveDARE(A, B, Q, R)

# The simulation starts, we can return the jth state with current state
def getLQRActionAtCurrentState(xcurrent):
   xk_bar = xref - xcurrent
   uk = Lk*xk_bar
   return uk
```

##### Selection of Q and R

With the help of the Q and R matrices, we can adjust the tradeoff between the performance and low energy. Having a bigger Q gain than R can make the system to increase the priority on performance while having a bigger R increases the low energy.

For now, the Q and R matrices are defined as below:

```math
Q = identity(6,6) / \text{max state values}\\[10pt]
R = identity(3,3) / \text{max action values}\\[10pt]
```

##### Solving the riccati equation (Solving DARE)

The Python Control Library has been used for its built-in Riccati Solver.

<https://python-control.readthedocs.io/en/0.9.0/generated/control.lqr.html>

<https://github.com/python-control/Slycot>

## Screenshots

### Open loop-constant input responses

* __No input, U=[0, 0, 0]__

Note: As expected, for the stochastic systems, gaussian process noise make fluctuations in the state although there is no input.

!["Constant input of U=[0, 0, 0]"](images/OL-0_0_0.png "Constant input of U=[0, 0, 0]")

### Feedback Control Examples

* __PID Reference Tracking__

All references and states:

!["PID example simulation, all references and states"](images/PID_Controller_plot_all_with_reference.png "PID example simulation, all references and states")

Roll reference and current angle:

!["PID example simulation, only the roll reference and current angle"](images/PID_Controller_plot_only_specific_element.png "PID example simulation, only the roll reference and current angle")

Reward Plot(Inverse/Negative of cost):

!["PID example simulation, Reward Plot(Inverse/Negative of cost)"](images/PID_Controller_plot_reward.png "PID example simulation, Reward Plot(Inverse/Negative of cost)")

Actions:

!["PID example simulation, Actions"](images/PID_Controller_plot_actions.png "PID example simulation, Actions")

* __LQR Reference Tracking__

All references and states:

!["LQR example simulation, all references and states"](images/LQR_plot_all_with_reference.png "LQR example simulation, all references and states")

Roll reference and current angle:

!["LQR example simulation, only the roll reference and current angle"](images/LQR_plot_only_specific_element.png "LQR example simulation, only the roll reference and current angle")

Reward Plot(Inverse/Negative of cost):

!["LQR example simulation, Reward Plot(Inverse/Negative of cost)"](images/LQR_plot_reward.png "LQR example simulation, Reward Plot(Inverse/Negative of cost)")

Actions:

!["LQR example simulation, Actions"](images/LQR_plot_actions.png "LQR example simulation, Actions")

## Installing the python packages with Docker(also with vscode devcontainers)

### Using Vscode Dev Container Extension

1- Open the project with vscode

2- Install the extention named "Remote - Containers"

3- The extension automatically asks that if you want to open this folder in dev container, click yes. (Or press f1 and search for 'Remote Containers: Rebuild Container and Open')

4- It will automatically install the required packages in the dev container, note that the image size will be ~7-8 GB.

### Using Classic Docker

There is a Dockerfile in the path `.devcontainer/Dockerfile` which you can build and start the ready to use dockerfile.

```
cd .devcontainer/

docker build -t akad/quad_rotational_simulation:v0 .

docker run -it --rm --init --gpus all --device /dev/nvidia0 --device /dev/nvidia-uvm --device /dev/nvidia-uvm-tools --device /dev/nvidiactl --net "host" -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix akad/quad_rotational_simulation:v0
```
## Required python packages

```bash
# For Environment
pip install gym matplotlib multiprocess

# For LQR Controller
sudo apt install cmake gfortran libopenblas-dev libgfortran3
pip install slycot control

# For MPC Controller
pip install do-mpc

# For RL Controller
pip install stable_baselines3 tensorboard
# For Linter(Not required for most of the purposes)
pip install flake8
```

## Running the tensorboard to observe the learning

`tensorboard --logdir ./logs/quad_tensorboard/`

## Executing the tests

To run the test codes, please execute this command:
`cd quad_rotational_simulation`
`export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/controllers/hsl/lib"`
`python3 test.py`

## Gitlab CI

<http://10.1.46.207/help/ci/quick_start/README>
<https://docs.gitlab.com/runner/install/linux-manually.html>

```bash
wget https://gitlab-runner-downloads.s3.amazonaws.com/latest/deb/gitlab-runner_amd64.deb
sudo dpkg -i gitlab-runner_amd64.deb
# Register the runner with the url and token information which can be found at 
# http://10.1.46.207/akad/python/environment/quad_rotational_simulation/-/settings/ci_cd
# The name of this runner given to be omtam-gitlab
gitlab-runner register
```

Replace this code below with using this command: `gedit ~/.gitlab-runner/config.toml`
<https://beenje.github.io/blog/posts/gitlab-ci-and-conda/>


```toml
concurrent = 1
check_interval = 0

[session_server]
  session_timeout = 1800

[[runners]]
  environment = ["https_proxy=http://10.1.200.207:8080", "http_proxy=http://10.1.200.207:8080", "HTTPS_PROXY=http://10.1.200.207:8080", "HTTP_PROXY=http://10.1.200.207:8080", "no_proxy=10.1.46.207", "NO_PROXY=10.1.46.207"]
  name = "bhdemirbilek-runner"
  url = "http://10.1.46.207/"
  token = "gbVZv1EBxPkYsms6y9HA"
  executor = "docker"
  [runners.custom_build_dir]
  [runners.cache]
   [runners.cache.s3]
   [runners.cache.gcs]
   [runners.cache.azure]
  [runners.docker]
   tls_verify = false
   image = "continuumio/miniconda3:latest"
   privileged = false
   disable_entrypoint_overwrite = false
   oom_kill_disable = false
   disable_cache = false
   volumes = ["/cache", "/opt/cache/conda/pkgs:/opt/conda/pkgs:rw", "/opt/cache/pip:/opt/cache/pip:rw"]
   shm_size = 0

```

```bash
gitlab-runner run
```
