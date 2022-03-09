# Quadcopter rotational system dynamics formulation, both nonlinear and linear representations


This study focuses on designing and developing controllers to control angular rates of a quadcopter. Therefore, only the rotational dynamics of the quadcopter are taken into consideration.
Below, you can see the assumptions made before a mathematical model:

* The rotational motion of the quadcopter is independent of its translational motion.
* The center of gravity coincides with the origin of the body-fixed frame.
* The structure of the quadcopter is rigid and symmetrical with the four arms coinciding with the body x- and y-axes.
* Drag and thrust forces are proportional to the square of the propellers speed.
* The propellers are rigid.

## Citing the Project

To cite this repository in publications:

https://arxiv.org/abs/2202.07021

Bibtex:

```bibtex
@article{demirbilek2022quadsim,
  title={QuadSim: A Quadcopter Rotational Dynamics Simulation Framework For Reinforcement Learning Algorithms},
  author={Demirbilek, Burak Han},
  journal={arXiv preprint arXiv:2202.07021},
  year={2022}
}
```

## Mathematical Formulation, Definitions, Dynamics

### Coordinate Frames

Coordinate frames are needed to describe the motions of the quadcopter before the quadcopter is mathematically modeled. Earth Fixed Frame W and Body Fixed Frame B is defined like this:

<p align="center">
  <img src="images/CoordinateFrames.png" alt="Coordinate Frame">
</p>

The earth-fixed frame is taken as the reference frame using the NED (North East Down) convention where the x-axis of the frame is pointed to the north.

The orientation of the quadcopter, known as attitude is expressed in the body-fixed frame by euler angles phi, theta, psi which corresponds to the roll, pitch and yaw angles.

In order to relate the orientation of the quadcopter in the earth-fixed frame, a rotation matrix is defined. Below, the rotation matrix R is defined, it converts from the body-fixed frame to the earth-fixed frame:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\bold{R}&space;=\begin{bmatrix}&space;&space;&space;C_{\psi}C_{\theta}&space;&&space;C_{\psi}S_{\theta}S_{\phi}&space;-&space;S_{\psi}C_{\phi}&space;&&space;C_{\psi}S_{\theta}C_{\phi}&space;&plus;&space;S_{\psi}S_{\phi}&space;\\&space;&space;&space;&space;&space;&space;S_{\psi}C_{\theta}&space;&&space;S_{\psi}S_{\theta}S_{\phi}&space;&plus;&space;C_{\psi}C_{\phi}&space;&&space;S_{\psi}S_{\theta}C_{\phi}&space;-&space;C_{\psi}S_{\phi}&space;\\&space;&space;&space;-S_{\theta}&space;&&space;C_{\theta}S_{\phi}&space;&&space;C_{\theta}C_{\phi}&space;\\\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \bold{R} =\begin{bmatrix} C_{\psi}C_{\theta} & C_{\psi}S_{\theta}S_{\phi} - S_{\psi}C_{\phi} & C_{\psi}S_{\theta}C_{\phi} + S_{\psi}S_{\phi} \\ S_{\psi}C_{\theta} & S_{\psi}S_{\theta}S_{\phi} + C_{\psi}C_{\phi} & S_{\psi}S_{\theta}C_{\phi} - C_{\psi}S_{\phi} \\ -S_{\theta} & C_{\theta}S_{\phi} & C_{\theta}C_{\phi} \\\end{bmatrix}" />

With the rotation matrix, you can calculate the earth-fixed position of any given position.

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\bold{v^{'}}&space;=&space;\bold{Rv}\\[10pt]\bold{v}&space;=&space;\begin{bmatrix}x\\y\\z\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\bold{v^{'}} = \bold{Rv}\\[10pt]\bold{v} = \begin{bmatrix}x\\y\\z\end{bmatrix}" />

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

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\U_{1}&space;=&space;db(w_{4}^2-w_{2}^2)&space;\\U_{2}&space;=&space;db(w_{1}^2-w_{3}^2)&space;\\U_{3}&space;=&space;k(w_{1}^2&plus;w_{3}^2-w_{2}^2-w_{4}^2)" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\U_{1} = db(w_{4}^2-w_{2}^2) \\U_{2} = db(w_{1}^2-w_{3}^2) \\U_{3} = k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2)" />

Where,

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\b&space;\text{&space;-&space;thrust&space;coefficient}&space;\\k&space;\text{&space;-&space;aerodrag&space;coefficient}&space;\\d&space;\text{&space;-&space;moment&space;arm}&space;\\w&space;\text{&space;-&space;motor&space;speed}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\b \text{ - thrust coefficient} \\k \text{ - aerodrag coefficient} \\d \text{ - moment arm} \\w \text{ - motor speed}" />

We can now define the nonlinear system dynamics as:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\ddot{\phi}&space;=&space;&space;\frac{(I_{yy}-&space;I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}}&space;&plus;&space;\frac{U_{1}}{I_{xx}}&space;\\\ddot{\theta}&space;=&space;&space;\frac{(I_{zz}-&space;I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}}&space;&plus;&space;\frac{U_{2}}{I_{yy}}&space;\\\ddot{\psi}&space;=&space;&space;\frac{(I_{xx}-&space;I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}}&space;&plus;&space;\frac{U_{3}}{I_{zz}}&space;\\" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\ddot{\phi} = \frac{(I_{yy}- I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}} + \frac{U_{1}}{I_{xx}} \\\ddot{\theta} = \frac{(I_{zz}- I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}} + \frac{U_{2}}{I_{yy}} \\\ddot{\psi} = \frac{(I_{xx}- I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}} + \frac{U_{3}}{I_{zz}} \\" />

For both linear and nonlinear systems, below you can find the state and input definitions:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;x&space;=&space;\begin{bmatrix}&space;&space;&space;\phi&space;\\&space;&space;&space;\dot{\phi}&space;\\&space;&space;&space;\theta&space;\\&space;&space;&space;\dot{\theta}&space;\\&space;&space;&space;\psi&space;\\&space;&space;&space;\dot{\psi}&space;\\\end{bmatrix},&space;\dot{x}&space;=&space;\begin{bmatrix}&space;&space;&space;\dot{\phi}&space;\\&space;&space;&space;\ddot{\phi}&space;\\&space;&space;&space;\dot{\theta}&space;\\&space;&space;&space;\ddot{\theta}&space;\\&space;&space;&space;\dot{\psi}&space;\\&space;&space;&space;\ddot{\psi}&space;\\\end{bmatrix}\\[10pt]u&space;=\begin{bmatrix}&space;&space;&space;U1&space;\\&space;&space;&space;U2&space;\\&space;&space;&space;U3&space;\\\end{bmatrix}=\begin{bmatrix}&space;&space;&space;db(w_{4}^2-w_{2}^2)&space;\\&space;&space;&space;db(w_{1}^2-w_{3}^2)&space;\\&space;&space;&space;k(w_{1}^2&plus;w_{3}^2-w_{2}^2-w_{4}^2)&space;\\\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} x = \begin{bmatrix} \phi \\ \dot{\phi} \\ \theta \\ \dot{\theta} \\ \psi \\ \dot{\psi} \\\end{bmatrix}, \dot{x} = \begin{bmatrix} \dot{\phi} \\ \ddot{\phi} \\ \dot{\theta} \\ \ddot{\theta} \\ \dot{\psi} \\ \ddot{\psi} \\\end{bmatrix}\\[10pt]u =\begin{bmatrix} U1 \\ U2 \\ U3 \\\end{bmatrix}=\begin{bmatrix} db(w_{4}^2-w_{2}^2) \\ db(w_{1}^2-w_{3}^2) \\ k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2) \\\end{bmatrix}" />

### State space(linear) representation of the quadcopter system

A LTI state space model is written below for model analysis and numerical calculations.

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\dot{x}=&space;Ax&space;&plus;&space;Bu\\y&space;=&space;Cx&space;&plus;&space;Du" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\dot{x}= Ax + Bu\\y = Cx + Du" />

The system was linearized at the hover position and system can fully observable

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;A&space;=&space;\begin{bmatrix}&space;&space;&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\end{bmatrix},B&space;=\begin{bmatrix}&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;\frac{1}{I_{xx}}&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;\frac{1}{I_{yy}}&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;\frac{1}{I_{zz}}&space;&space;\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} A = \begin{bmatrix} 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix},B =\begin{bmatrix} 0 & 0 & 0 \\ \frac{1}{I_{xx}} & 0 & 0 \\ 0 & 0 & 0 \\ 0 & \frac{1}{I_{yy}} & 0 \\ 0 & 0 & 0 \\ 0 & 0 & \frac{1}{I_{zz}} \end{bmatrix}" />

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;C&space;=&space;\begin{bmatrix}&space;&space;&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix},D&space;=&space;\begin{bmatrix}&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\\&space;&space;&space;0&space;&&space;0&space;&&space;0&space;\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} C = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix},D = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}" />

Where the state vector x and the input vector u is defined like this:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;x&space;=&space;\begin{bmatrix}&space;&space;&space;\phi&space;\\&space;&space;&space;\dot{\phi}&space;\\&space;&space;&space;\theta&space;\\&space;&space;&space;\dot{\theta}&space;\\&space;&space;&space;\psi&space;\\&space;&space;&space;\dot{\psi}&space;\\\end{bmatrix},&space;\dot{x}&space;=&space;\begin{bmatrix}&space;&space;&space;\dot{\phi}&space;\\&space;&space;&space;\ddot{\phi}&space;\\&space;&space;&space;\dot{\theta}&space;\\&space;&space;&space;\ddot{\theta}&space;\\&space;&space;&space;\dot{\psi}&space;\\&space;&space;&space;\ddot{\psi}&space;\\\end{bmatrix}\\[10pt]u&space;=\begin{bmatrix}&space;&space;&space;U1&space;\\&space;&space;&space;U2&space;\\&space;&space;&space;U3&space;\\\end{bmatrix}=\begin{bmatrix}&space;&space;&space;db(w_{4}^2-w_{2}^2)&space;\\&space;&space;&space;db(w_{1}^2-w_{3}^2)&space;\\&space;&space;&space;k(w_{1}^2&plus;w_{3}^2-w_{2}^2-w_{4}^2)&space;\\\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} x = \begin{bmatrix} \phi \\ \dot{\phi} \\ \theta \\ \dot{\theta} \\ \psi \\ \dot{\psi} \\\end{bmatrix}, \dot{x} = \begin{bmatrix} \dot{\phi} \\ \ddot{\phi} \\ \dot{\theta} \\ \ddot{\theta} \\ \dot{\psi} \\ \ddot{\psi} \\\end{bmatrix}\\[10pt]u =\begin{bmatrix} U1 \\ U2 \\ U3 \\\end{bmatrix}=\begin{bmatrix} db(w_{4}^2-w_{2}^2) \\ db(w_{1}^2-w_{3}^2) \\ k(w_{1}^2+w_{3}^2-w_{2}^2-w_{4}^2) \\\end{bmatrix}" />

### Nonlinear dynamics

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\dot{x}&space;=&space;f(x,u)\\y&space;=&space;g(x,u)\\[30pt]\dot{x}&space;=&space;f(x,u)&space;=\begin{bmatrix}&space;&space;&space;\dot{\phi}&space;\\[10pt]&space;&space;&space;\frac{(I_{yy}-&space;I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}}&space;&plus;&space;\frac{U_{1}}{I_{xx}}&space;\\[10pt]&space;&space;&space;\dot{\theta}&space;\\[10pt]&space;&space;&space;\frac{(I_{zz}-&space;I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}}&space;&plus;&space;\frac{U_{2}}{I_{yy}}&space;\\[10pt]&space;&space;&space;\dot{\psi}&space;\\[10pt]&space;&space;&space;\frac{(I_{xx}-&space;I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}}&space;&plus;&space;\frac{U_{3}}{I_{zz}}&space;\\[10pt]\end{bmatrix}\\[20pt]y&space;=&space;g(x,u)&space;=&space;x&space;=&space;\begin{bmatrix}&space;&space;&space;\phi&space;\\&space;&space;&space;\dot{\phi}&space;\\&space;&space;&space;\theta&space;\\&space;&space;&space;\dot{\theta}&space;\\&space;&space;&space;\psi&space;\\&space;&space;&space;\dot{\psi}&space;\\\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\dot{x} = f(x,u)\\y = g(x,u)\\[30pt]\dot{x} = f(x,u) =\begin{bmatrix} \dot{\phi} \\[10pt] \frac{(I_{yy}- I_{zz})\dot{\theta}\dot{\psi}}{I_{xx}} + \frac{U_{1}}{I_{xx}} \\[10pt] \dot{\theta} \\[10pt] \frac{(I_{zz}- I_{xx})\dot{\phi}\dot{\psi}}{I_{yy}} + \frac{U_{2}}{I_{yy}} \\[10pt] \dot{\psi} \\[10pt] \frac{(I_{xx}- I_{yy})\dot{\phi}\dot{\theta}}{I_{zz}} + \frac{U_{3}}{I_{zz}} \\[10pt]\end{bmatrix}\\[20pt]y = g(x,u) = x = \begin{bmatrix} \phi \\ \dot{\phi} \\ \theta \\ \dot{\theta} \\ \psi \\ \dot{\psi} \\\end{bmatrix}" />

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
   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;x&space;=&space;&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;\phi_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\phi}_{err}&space;\\&space;&space;&space;&space;&space;&space;\theta_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\theta}_{err}&space;\\&space;&space;&space;&space;&space;&space;\psi_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\psi}_{err}&space;\\&space;&space;&space;\end{bmatrix}&space;" title="http://latex.codecogs.com/gif.latex?\dpi{110} x = \begin{bmatrix} \phi_{err} \\ \dot{\phi}_{err} \\ \theta_{err} \\ \dot{\theta}_{err} \\ \psi_{err} \\ \dot{\psi}_{err} \\ \end{bmatrix} " />

Note that error variables are the difference between the reference state and current state.

The angle states are also mapped between [-pi, pi).

##### Actions

   The actions are defined as the original definition, the action is a vector with size `(3,)` and it tells the torque/moment vector acting on the quadcopter in the body-fixed frame.

   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;x&space;=&space;&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;\phi_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\phi}_{err}&space;\\&space;&space;&space;&space;&space;&space;\theta_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\theta}_{err}&space;\\&space;&space;&space;&space;&space;&space;\psi_{err}&space;\\&space;&space;&space;&space;&space;&space;\dot{\psi}_{err}&space;\\&space;&space;&space;\end{bmatrix}&space;&space;&space;&space;U&space;=&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;&space;&space;&space;u_{1}&space;\\&space;&space;&space;&space;&space;&space;&space;&space;&space;u_{2}&space;\\&space;&space;&space;&space;&space;&space;&space;&space;&space;u_{3}&space;\\&space;&space;&space;\end{bmatrix}&space;&space;&space;=&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;&space;&space;&space;\tau_{\phi}&space;\\&space;&space;&space;&space;&space;&space;&space;&space;&space;\tau_{\theta}&space;\\&space;&space;&space;&space;&space;&space;&space;&space;&space;\tau_{\psi}&space;\\&space;&space;&space;\end{bmatrix}" title="http://latex.codecogs.com/gif.latex?\dpi{110} x = \begin{bmatrix} \phi_{err} \\ \dot{\phi}_{err} \\ \theta_{err} \\ \dot{\theta}_{err} \\ \psi_{err} \\ \dot{\psi}_{err} \\ \end{bmatrix} U = \begin{bmatrix} u_{1} \\ u_{2} \\ u_{3} \\ \end{bmatrix} = \begin{bmatrix} \tau_{\phi} \\ \tau_{\theta} \\ \tau_{\psi} \\ \end{bmatrix}" />

   The environment will automatically calculate the propeller speeds with its motor mixing algorithm. So the agent only needs to give the environment the torque vector.

#### Environment Limits(min and max)

   After defining states and actions, the hard limits of these needs to be calculated. These limits needs to be defined in the Gym environment class.

   From the datasheet of the EMAX 650kv dc motors, the maximum speed rating of a dc motor is 4720rpm.
   To calculate the minimum speed rating, we need to calculate the minimum thrust for a quadcopter to hover its weight.


   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\m=1.587\\&space;&space;&space;g=9.81\\&space;&space;&space;F=m*g=15.57N" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\m=1.587\\ g=9.81\\ F=m*g=15.57N" />


   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\text{Thrust}&space;=&space;&space;&space;&space;b\sum_{i=1}^{4}w_{i}^{2}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \text{Thrust} = b\sum_{i=1}^{4}w_{i}^{2}" />


   Therefore, to achieve the minimum trust, the motor speeds needs to be

   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;w_{min}&space;=&space;w_{i}&space;=&space;\sqrt{\frac{15.57}{4&space;*&space;4.0687&space;*&space;10^{-7}}}&space;\approx&space;3093&space;\text{&space;rpm}\\[10pt]&space;&space;&space;w_{max}&space;=&space;4720&space;\text{&space;rpm&space;(from&space;motor&space;datasheet)}" title="http://latex.codecogs.com/gif.latex?\dpi{110} w_{min} = w_{i} = \sqrt{\frac{15.57}{4 * 4.0687 * 10^{-7}}} \approx 3093 \text{ rpm}\\[10pt] w_{max} = 4720 \text{ rpm (from motor datasheet)}" />

   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\rad/s&space;=&space;\frac{RPM*2\pi}{60}\\[10pt]&space;&space;&space;w_{max}&space;=&space;494.28&space;\text{&space;rad/s}\\[10pt]&space;&space;&space;w_{min}&space;=&space;323.90&space;\text{&space;rad/s}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\rad/s = \frac{RPM*2\pi}{60}\\[10pt] w_{max} = 494.28 \text{ rad/s}\\[10pt] w_{min} = 323.90 \text{ rad/s}" />

   From the motor min-max speed values, we can calculate the U min and max values:


   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\U_{max}&space;=&space;&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;U_{1}\\&space;&space;&space;&space;&space;&space;U_{2}\\&space;&space;&space;&space;&space;&space;U_{3}\\&space;&space;&space;\end{bmatrix}&space;&space;&space;=&space;&space;&space;\begin{bmatrix}&space;&space;&space;&space;&space;&space;db((w^2_{max})-(w^2_{min}))&space;\\&space;&space;&space;&space;&space;&space;db((w^2_{max})-(w^2_{min}))&space;\\&space;&space;&space;&space;&space;&space;2k((w^2_{max})-(w^2_{min}))&space;\\&space;&space;&space;\end{bmatrix}\\[10pt]&space;&space;&space;U_{min}&space;=&space;-U_{max}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\U_{max} = \begin{bmatrix} U_{1}\\ U_{2}\\ U_{3}\\ \end{bmatrix} = \begin{bmatrix} db((w^2_{max})-(w^2_{min})) \\ db((w^2_{max})-(w^2_{min})) \\ 2k((w^2_{max})-(w^2_{min})) \\ \end{bmatrix}\\[10pt] U_{min} = -U_{max}" />


   After that, we can calculate maximum state values when the motor speeds are constant by calculating the definite integral over this period. Please note that rotational values are already bounded, so we dont need to calculate the maximum value of them.

 
   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\phi_{max}&space;=&space;\pi&space;\quad&space;\text{&space;rad}\\[10pt]&space;&space;&space;\dot{\phi}_{max}=&space;\frac{U_{1max}}{Ixx}*(t_{end}-t_{start})&space;\quad&space;\text{&space;rad/s}\\[10pt]&space;&space;&space;\theta_{max}=&space;\pi&space;\quad&space;\text{&space;rad}\\[10pt]&space;&space;&space;\dot{\theta}_{max}&space;=&space;\frac{U_{2max}}{Iyy}*(t_{end}-t_{start})&space;\quad&space;\text{&space;rad/s}\\[10pt]&space;&space;&space;\psi_{max}&space;=&space;\pi&space;&space;\quad&space;\text{&space;rad}\\[10pt]&space;&space;&space;\dot{\psi}_{max}&space;=&space;&space;\frac{U_{3max}}{Izz}*(t_{end}-t_{start})&space;\quad&space;\text{&space;rad/s}\\[10pt]" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\phi_{max} = \pi \quad \text{ rad}\\[10pt] \dot{\phi}_{max}= \frac{U_{1max}}{Ixx}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt] \theta_{max}= \pi \quad \text{ rad}\\[10pt] \dot{\theta}_{max} = \frac{U_{2max}}{Iyy}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt] \psi_{max} = \pi \quad \text{ rad}\\[10pt] \dot{\psi}_{max} = \frac{U_{3max}}{Izz}*(t_{end}-t_{start}) \quad \text{ rad/s}\\[10pt]" />


#### Control Algorithm and Simulation Frequency

   For this simulation, a simulation frequency of __250Hz__ and controller(control algorithm) frequency of __50Hz__ have been selected and used.

   From the openai gym perspective, the action selected from the algorithm will be constant for every 0.02(1/50) seconds.

#### Rewards/Costs

   For the reward function, the quadratic cost of error(reference-current) has been used.


   <img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\overline{x}&space;=&space;x_{ref}-x_{current}&space;\\[10pt]&space;&space;&space;&space;&space;&space;cost&space;=&space;\overline{x}^{T}Q\overline{x}&space;&plus;&space;u^{T}Ru&space;\\[10pt]&space;&space;&space;&space;&space;&space;Q&space;=&space;identity(6,6)&space;/&space;\text{max&space;state&space;values}&space;\\[10pt]&space;&space;&space;&space;&space;&space;R&space;=&space;identity(3,3)&space;/&space;\text{max&space;action&space;values}&space;\\[10pt]&space;&space;&space;\text{reward}&space;=&space;-\text{cost}&space;\\" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\overline{x} = x_{ref}-x_{current} \\[10pt] cost = \overline{x}^{T}Q\overline{x} + u^{T}Ru \\[10pt] Q = identity(6,6) / \text{max state values} \\[10pt] R = identity(3,3) / \text{max action values} \\[10pt] \text{reward} = -\text{cost} \\" />

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

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\begin{tabular}{&space;|c|c|&space;}&space;&space;&space;\hline&space;&space;\textbf{Parameter}&space;&&space;\textbf{Default&space;Value}&space;&space;\\&space;&space;&space;\hline&space;&space;Moment&space;of&space;inertia&space;about&space;x-axis,&space;Ixx&space;&&space;0.0213&space;kg$m^2$&space;&space;\\&space;&space;&space;\hline&space;&space;Moment&space;of&space;inertia&space;about&space;y-axis,&space;Iyy&space;&&space;0.02217&space;kg$m^2$&space;\\&space;&space;&space;\hline&space;&space;Moment&space;of&space;inertia&space;about&space;z-axis,&space;Izz&space;&&space;0.0282&space;kg$m^2$&space;\\&space;&space;&space;\hline&space;&space;Mass,&space;m&space;&&space;1.587&space;kg&space;&space;\\&space;&space;&space;\hline&space;&space;Gravity,&space;g&space;&&space;9.81&space;N\\&space;&space;&space;\hline&space;&space;Moment&space;arm,&space;d&space;&&space;0.243&space;m&space;&space;\\&space;&space;&space;\hline&space;&space;Thrust&space;coefficient,&space;b&space;&&space;3.7102e-5&space;N$s^2$\\&space;&space;&space;\hline&space;&space;Drag&space;coefficient,&space;k&space;&&space;7.6933e-7&space;Nm$s^2$\\[5pt]&space;&space;\hline&space;&space;Propeller&space;maximum&space;angular&space;speed,&space;$w_{max}$&space;&&space;494.27&space;rad/s&space;\\[5pt]&space;&space;\hline&space;&space;Soft&space;phidot&space;limit,&space;$\dot{\phi}$&space;&&space;35&space;rad/s&space;\\&space;[5pt]&space;&space;\hline&space;&space;Soft&space;thetadot&space;limit,&space;$\dot{\theta}$&space;&&space;35&space;rad/s&space;\\&space;&space;&space;\hline&space;&space;Soft&space;psidot&space;limit,&space;$\dot{\psi}$&space;&space;&&space;35&space;rad/s&space;\\&space;&space;&space;\hline\end{tabular}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \begin{tabular}{ |c|c| } \hline \textbf{Parameter} & \textbf{Default Value} \\ \hline Moment of inertia about x-axis, Ixx & 0.0213 kg$m^2$ \\ \hline Moment of inertia about y-axis, Iyy & 0.02217 kg$m^2$ \\ \hline Moment of inertia about z-axis, Izz & 0.0282 kg$m^2$ \\ \hline Mass, m & 1.587 kg \\ \hline Gravity, g & 9.81 N\\ \hline Moment arm, d & 0.243 m \\ \hline Thrust coefficient, b & 3.7102e-5 N$s^2$\\ \hline Drag coefficient, k & 7.6933e-7 Nm$s^2$\\[5pt] \hline Propeller maximum angular speed, $w_{max}$ & 494.27 rad/s \\[5pt] \hline Soft phidot limit, $\dot{\phi}$ & 35 rad/s \\ [5pt] \hline Soft thetadot limit, $\dot{\theta}$ & 35 rad/s \\ \hline Soft psidot limit, $\dot{\psi}$ & 35 rad/s \\ \hline\end{tabular}" />

## Controllers

### PID Control Algorithm

A proportional-integral-derivative(PID) controller is a control loop mechanism for controlling the error signal(difference between the desired setpoint and measured variable) by applying proportional, integral, derivative terms to the system input.

The mathematical form of the overall control function is:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\u(t)&space;=&space;K_{p}e(t)&space;&plus;&space;K_{i}&space;\int_{0}^{t}e(\tau)d\tau&space;&plus;&space;K_{d}&space;\frac{de(t)}{dt}\\[10pt]\text{Where&space;}K_{p},&space;K_{i}\text{&space;and&space;}K_{d}\\[10pt]\text{&space;are&space;all&space;non-negative&space;proportional,&space;integral&space;and&space;derivative&space;terms&space;respectively.}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\u(t) = K_{p}e(t) + K_{i} \int_{0}^{t}e(\tau)d\tau + K_{d} \frac{de(t)}{dt}\\[10pt]\text{Where }K_{p}, K_{i}\text{ and }K_{d}\\[10pt]\text{ are all non-negative proportional, integral and derivative terms respectively.}" />

Since PID control algorithm only works for single-input, single-output systems, 3 parallel PID controllers are needed for roll, pitch, yaw angle control.

Matlab-Simulink has been used for auto tune functionality.

Below, the tuned parameters are listed:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\begin{tabular}{|c|c|}&space;&space;&space;\hline&space;&space;&space;Roll,&space;$K_{p}$&space;&space;&&space;4.72531916175911&space;\\&space;&space;&space;Roll,&space;$K_{i}$&space;&space;&&space;3.73086282471387&space;\\&space;&space;&space;Roll,&space;$K_{d}$&space;&space;&&space;1.49621161575424&space;\\[10pt]&space;&space;&space;\hline&space;&space;&space;Pitch,&space;$K_{p}$&space;&space;&&space;3.63871317561002&space;\\&space;&space;&space;Pitch,&space;$K_{i}$&space;&space;&&space;2.14232438611662&space;\\&space;&space;&space;Pitch,&space;$K_{d}$&space;&space;&&space;1.54507805402352&space;\\[10pt]&space;&space;&space;\hline&space;&space;&space;Yaw,&space;$K_{p}$&space;&space;&&space;4.6284037687056&space;\\&space;&space;&space;Yaw,&space;$K_{i}$&space;&space;&&space;2.72501342753779&space;\\&space;&space;&space;Yaw,&space;$K_{d}$&space;&space;&&space;1.96532255856848&space;\\&space;&space;&space;\hline\end{tabular}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \begin{tabular}{|c|c|} \hline Roll, $K_{p}$ & 4.72531916175911 \\ Roll, $K_{i}$ & 3.73086282471387 \\ Roll, $K_{d}$ & 1.49621161575424 \\[10pt] \hline Pitch, $K_{p}$ & 3.63871317561002 \\ Pitch, $K_{i}$ & 2.14232438611662 \\ Pitch, $K_{d}$ & 1.54507805402352 \\[10pt] \hline Yaw, $K_{p}$ & 4.6284037687056 \\ Yaw, $K_{i}$ & 2.72501342753779 \\ Yaw, $K_{d}$ & 1.96532255856848 \\ \hline\end{tabular}" />


### MPC Algorithm

Model predictive control (MPC) is an advanced method of process control that is used to control a process while satisfying a set of constraints. Model predictive controllers rely on dynamic models of the process, most often linear empirical models obtained by system identification.

The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots in account. This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot and then optimizing again, repeatedly, thus differing from Linear-Quadratic Regulator (LQR).

Also MPC has the ability to anticipate future events and can take control actions accordingly. PID controllers do not have this predictive ability.

Below, the tuned parameters are listed:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\N_{p}&space;=&space;50,\\[10pt]t_{step}&space;=&space;0.02,\\[10pt]n_{robust}&space;=&space;1,\\[10pt]t_{end}&space;=&space;10,\\[10pt]linear&space;solver&space;=&space;MA27,\\[10pt]" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\N_{p} = 50,\\[10pt]t_{step} = 0.02,\\[10pt]n_{robust} = 1,\\[10pt]t_{end} = 10,\\[10pt]linear solver = MA27,\\[10pt]" />

### Infinite Horizon Discrete Linear Quadratic Regulator (LQR)

For Linear Quadratic Gaussian Systems, the dynamics are described as below:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\x_{k&plus;1}=&space;Ax_{k}&space;&plus;&space;Bu_{k}&space;&plus;&space;Gw_{k}\\y_{k}&space;=&space;Cx_{k}&space;&plus;&space;Hv_{k}\\u_{k}&space;=&space;g_{k}(y_{0:k})\\[10pt]x_{0}&space;\sim&space;\mathcal{N}(x_{0};&space;\overline{x_{0}},&space;\Sigma_{0})\\w_{k}&space;\sim&space;\mathcal{N}(w_{k};&space;0,&space;\Sigma_{w})\\v_{k}&space;\sim&space;\mathcal{N}(v_{k};&space;0,&space;\Sigma_{v})\\[10pt]J=\sum_{k=0}^{N}c_{k}(x_{k},u_{k})\\[10pt]\text{where:&space;}\\[10pt]c_{k}(x_{k},u_{k})&space;\triangleq&space;x_{k}^{T}Q_{k}x_{k}&space;&plus;&space;u_{k}^{T}R_{k}u_{k},&space;\quad&space;0\leq&space;k\leq&space;N-1&space;\\[10pt]c_{N}(x_{N},u_{N})&space;=&space;c_{N}(x_{N})&space;=&space;x_{N}^{T}Q_{N}x_{N}\\[10pt]" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\x_{k+1}= Ax_{k} + Bu_{k} + Gw_{k}\\y_{k} = Cx_{k} + Hv_{k}\\u_{k} = g_{k}(y_{0:k})\\[10pt]x_{0} \sim \mathcal{N}(x_{0}; \overline{x_{0}}, \Sigma_{0})\\w_{k} \sim \mathcal{N}(w_{k}; 0, \Sigma_{w})\\v_{k} \sim \mathcal{N}(v_{k}; 0, \Sigma_{v})\\[10pt]J=\sum_{k=0}^{N}c_{k}(x_{k},u_{k})\\[10pt]\text{where: }\\[10pt]c_{k}(x_{k},u_{k}) \triangleq x_{k}^{T}Q_{k}x_{k} + u_{k}^{T}R_{k}u_{k}, \quad 0\leq k\leq N-1 \\[10pt]c_{N}(x_{N},u_{N}) = c_{N}(x_{N}) = x_{N}^{T}Q_{N}x_{N}\\[10pt]" />

In the complete information case, C is the identity matrix, D is zero and y will be:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;y_{k}=x_{k}&space;\quad&space;\forall&space;k" title="http://latex.codecogs.com/gif.latex?\dpi{110} y_{k}=x_{k} \quad \forall k" />

For LQR control, Sigma w and Sigma v are also needs to be 0, which means that systems are deterministic.

Also, the matrix Q needs to be positive semi definite(>=0) and the matrix R needs to be positive definite (>0).

#### Regulation task

In the Linear Quadratic Regulator Control (LQR), we can define the optimal feedback policy with given cost function J to regulate the states and inputs to zero.

We can calculate the optimal policy for a finite horizon N with the equations below:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\overline{W}&space;=&space;A^{T}(\overline{W}&space;-&space;\overline{W}&space;B(B^{T}\overline{W}B&space;&plus;&space;R)^{-1}&space;A&space;&plus;&space;Q&space;\\[10pt]\overline{L}&space;=&space;(B^{T}\overline{W}B&space;&plus;&space;R)^{-1}&space;B^{T}\overline{W}A&space;\\[10pt]\bold{&space;&space;&space;u_{k}^{*}&space;=&space;-\overline{L}x_{k}}&space;\\[10pt]" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\overline{W} = A^{T}(\overline{W} - \overline{W} B(B^{T}\overline{W}B + R)^{-1} A + Q \\[10pt]\overline{L} = (B^{T}\overline{W}B + R)^{-1} B^{T}\overline{W}A \\[10pt]\bold{ u_{k}^{*} = -\overline{L}x_{k}} \\[10pt]" />

The dimensions of these matrix as follows:

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\A&space;=&space;(6,6)\\B&space;=&space;(6,3)\\[10pt]Q&space;=&space;(6,6)\\R&space;=&space;(3,3)\\[10pt]W&space;=&space;(6,6)\\L&space;=&space;(3,6)\\" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\A = (6,6)\\B = (6,3)\\[10pt]Q = (6,6)\\R = (3,3)\\[10pt]W = (6,6)\\L = (3,6)\\" />

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

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\\bold{\overline{x_{k}}&space;=&space;x_{ref}&space;-&space;x_{k}}\\[10pt]\bold{u_{k}^{*}&space;=&space;\overline{L}\overline{x_{k}}}" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\\bold{\overline{x_{k}} = x_{ref} - x_{k}}\\[10pt]\bold{u_{k}^{*} = \overline{L}\overline{x_{k}}}" />

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

<img src="http://latex.codecogs.com/gif.latex?\dpi{110}&space;\\Q&space;=&space;identity(6,6)&space;/&space;\text{max&space;state&space;values}\\[10pt]R&space;=&space;identity(3,3)&space;/&space;\text{max&space;action&space;values}\\[10pt]" title="http://latex.codecogs.com/gif.latex?\dpi{110} \\Q = identity(6,6) / \text{max state values}\\[10pt]R = identity(3,3) / \text{max action values}\\[10pt]" />

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

## Installation

### Required python packages
```
conda create -n quadsim python=3.7 -y
conda activate quadsim

conda install pytorch torchvision torchaudio cudatoolkit=11.3 -c pytorch
conda install -c conda-forge gym matplotlib multiprocess slycot control stable-baselines3 tensorboard flake8 casadi libgfortran==3.0.0 -y

pip install do-mpc
```

### Running the tensorboard to observe the learning

`tensorboard --logdir ./logs/quad_tensorboard/`

### Executing the tests

To run the test codes, please execute this command:
```bash
cd quad_rotational_simulation
conda activate quadsim
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/src/controllers/hsl/lib"
python tests/unit_tests.py
python tests/test_pid.py
python tests/test_lqr.py
python tests/test_lqg.py
python tests/test_mpc_linear.py
python tests/test_mpc_nonlinear.py
python tests/test_rl_a2c.py
python tests/test_rl_td3.py
python tests/test_rl_ppo.py
python tests/test_rl_ddpg.py
python tests/test_rl_sac.py
```

