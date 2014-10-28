# Dynamic Simulation

To evaluate the generated trajectories a simulator for the dynamics was developed.
The simulator was build on the \name{SimDynamics} framework that is part of \name{Simox}.
\name{SimDynamics} uses \name{Bullet Physics} as underlying physics framework.
A big part of the work on the simulator was spend on configuring the parameters and finding flaws in the physics simulation.
Thus, the simulator includes a extensive logging and visualization framework that measures all important parameters of the simulation.
For visualizing and analysing the measurement the Open Source tools \name{IPython}, \name{numpy} and \name{pandas} where used.

## Simulating rigid body dynamics {#section:rigid-body-simulation}

Physical simulation in general can be divided into discrete methods and continuous methods.
Discrete simulators only compute the state of the system at specific points in time, while
continuous simulators are able to compute the state of the system at any point in time.
While continuous simulation is the more flexible approach, it quickly becomes impractical
with a high number of constraints. A large amount of differential equations
need to be solved. Since it is hard to obtain analytical solutions for most differential equations,
numerical methods need to be used, which are slow.
In contrast, discrete simulation methods only compute simulation values for specific time steps.
This exploits the observation that we will typically query the state of the physics engine only
at a fixed rate anyway, e.g. at each iteration of our control loop.
Rather than solving the differential equations that describe the physical system in each step,
a solution is derived from the previous simulation state.

In a physical system we can typically find two kind of forces: Applied forces and constraint forces.
Applied forces are the input forces of the system. Source of applied forces are for example gravity and objects like springs.
Constraint forces are fictional forces that arise from constraints we impose on the system:
Non-penetration constraints, friction constraints, position constraints of joints or velocity constraints
for motors.
Mathematically we can express such constraints in the form: $C(x) = 0$ or $\dot{C}(x) = 0$ in the case of equality constraints,
or as $C(x) \geq 0$ or $\dot{C(x)} \geq 0$ in the case of inequality constraints. $x$ is typically a vector in Cartesian space.

For example the position constraint of a joint $p$ connected to a base $p_0$ with distance $r_0 = ||p-p_0||$
would be: $C(p) = || p - p_0 ||^2 - r_0^2$
If $p$ is moving with a linear velocity $v$ a constraint force $F_c$ is applied to $p$ to maintain this constraint.
We can view $C$ as a transformation from our Cartesian space to the constraint space. Thus, by computing the Jacobian
$J$ of $C$ we can relate velocities in both spaces.
Furthermore, we can relate constraint space forces $\lambda$ with Cartesian space forces using the transpose of the Jacobian.
Thus, if we can find the constraint space force $\lambda$ that is needed to maintain this constraint, we can compute $F_c$ using $F_c = J^T \lambda$.
Computing this constraint space forces is the task of the constraint solver.

The constrained solver used by \name{Bullet}, and thus the constraint solver used for simulating the
patterns in this thesis, is a sequential impulse solver.
To make some calculations easier, a SI solver works with impulses and velocities, rather than forces and accelerations.
Impulses and forces can be easily transformed in each other as $P = F \cdot T$ where $P$ is the impulse and $T$ the time step size.
A sequential impulse solver tries to compute the constraint force (in this case rather impulse) $\lambda$ for each constraint *separately*.
For each constraint the following steps are executed:

1. Compute the velocity that results from *applied forces* on the body
2. Calculate constraint force to satisfy the velocity constraint
3. Compute new velocity resulting from constraint force *and* applied force on the body
4. Update position of the body by integrating velocity: $p[n+1] = p[n] + v \cdot T$

Of course this might not lead to a global solution, as satisfying a constraint might violate a previously solved one.
The idea is to repeatedly loop over all constraints, so that a global solution will be reached.
The quality of this method relies on how often this loop is executed. Consider the case of a kinematic
chain. Moving a link will always violate at least one position constraint. A lot of iterations are needed
to yield good results in this case.
It becomes even worse in the case of a parallel kinematic chain, that is in contact with the ground.
This is the case for a bipedal robot in dual support stance.
Solving a non-penetration constrain on either end will invalidate the position constraint of the next link.
In turn, the position constraint of each link needs to be updated until the other end of the kinematic chain is reached. If the non-penetration
constraint is violated again for this end, the whole process starts again in reverse direction. This leads to oscillations that need a high number
 of iterations to level off to an acceptable level.
Despite these inaccuracies, by using enough solver iterations an overall usable systems can be derived. However, the velocities will still have
a small random error in each simulation step.
This poses a major problem when trying to measure accelerations, as the random error causes them to accelerate wildly.
This circumstance needs to be taken into account when dealing with values derived from the acceleration (e.g. the ZMP),
as mean-filters might be necessary.

## Practical challenges of physics simulations

While walking only the feet of the robot are in contact with the ground. Thus, the stability of the whole robots depends on the contact of the feet with the floor. Especially in single support phase that area is very small with regard to the size of the robot.
For that reason the accuracy of ground contact forces and friction is of utmost importance for the quality of the simulation.
In general, three classes of errors need to be eliminated to get a good simulation:

1. Incorrectly configured parameters, such as fictions coefficients and contact thresholds

2. Numerical errors

3. Inherent errors of the method

As outlined in section \ref{section:rigid-body-simulation}, the physics of the system
are formulated as input forces and constraints that need to be solved for the constraint forces.
Since \name{Bullet} uses the iterative approach described in section \ref{section:rigid-body-simulation}, it is important
to use a sufficient amount of iterations for each simulation step. Another important parameter is the time step of each simulation step.
Through experimental evaluation, a simulation with 2000 solver iterations and a time step size of $1 ms$ was sufficiently stable.
However, since the number of iterations is very high and a lot of time steps are calculated during the simulation, numeric errors become significant.
That made is necessary to use double precision floating point numbers for the values used during simulation.

To decide which contact constraints are active for which points, \name{Bullet} must solve for object collisions. Depending on the objects
involved different algorithms are used to calculate the contact points. Major gains in accuracy could be observed by replacing
the feet and the floor with simple box shapes, instead using mesh based models.

## Implementation

\begin{figure*}[htb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/architechture.png}
\caption{Achitechture of the simulator}
\label{img:simulator-achitechture}
\end{figure*}

The simulator was designed to load arbitrary motions in the \name{MMM} format and replay them. Additional stabilization algorithms can be applied
depending on additional information provided in the \name{MMM} motions.
These stabilization algorithms are implemented by sub-classing ```TrajectoryController```. They are invoked in each simulation step.

Currently three controllers are implemented. A controller based on the stabilizer proposed by Kajita (as outlined in section \ref{section:stabilizer}),
a simple heuristic stabilizer (outlined in section \ref{section:alternative-approach})
and a controller that just plays back the specified walking pattern.
Each control algorithm is invoked in the same cycle time as the reference walking pattern.

The resulting, possibly modified, joint angles are then interpolated using cubic splines.
This ensures a smooth velocity profile and mitigates destabilizing oscillations caused by large velocity changes.
The interpolated angles are send via \name{SimDynamics} to the corresponding motors.
Since the motor implemented in \name{Bullet} are velocity controlled,
PID based motor controllers were added to \name{SimDynamics}.
They control the motor velocities to compensate position errors.
The motors implemented in \name{Bullet} do not limit the motor velocity and acceleration.
This is not consistent with real motors, thus limits for velocities and acceleration where introduced to \name{SimDynamics},
that can be configured per joint.

The graphical user-interface supports the visualization of measured and desired values
for CoM, Immediate Capture Point, ZMP, ankle torque, and ground reaction forces.
The best-case trajectory can be visualized as an overlay robot ("phantom robot").
Figure \ref{img:simulator-thumbs} shows some of the supported visualizations.

\begin{figure*}[htb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/simulator_thumbs.png}
\caption{GUI of the simulator. Top left: Before starting the physics engine. Top right: Robot with
best-case "phantom" overlay. Bottom left: Robot standing with support polygone visualization. Bottom right:
Close-up of the support polygone, ankle torques and ground reaction forces.}
\label{img:simulator-thumbs}
\end{figure*}

An important part of the simulation is the generation of measurements that can be carefully evaluated offline or displayed in the visualization.
For this purpose a modular measurement component was added to the simulator.
An important design goal was to keep the measurement component as simple to extend and maintain as possible.
Each module measures a specific set of values and writes them, indexed by the corresponding timestamp, to its log file.
As output format the well know plain text format CSV was used.
The visualization can query the measurement components directly to get the newest values to be displayed.
For example the ZMP module measures the actual ZMP and also provides an interface to query the trajectory ZMP and the reference ZMP that was provided as input for the pattern generator.
Thus all three values can be displayed in the visualization and easily compared later by analysing the log file.
Since the goal was to keep the component as simple as possible, we use existing well known tools for analyzing the generated log files.
Some small helper scripts are provided to make it easier to load the data into the time series analysis framework \name{pandas}.
\name{Pandas} interfaces with the popular plotting framework \name{matplotlib} to display plots of the data.
\name{IPython} is used to easily run the analysis and display the results in a browser window.
All plots of simulated patterns found in this thesis can be generated automatically for every simulation.

\begin{figure}[htb]
  \begin{center}
     \includegraphics[width=\textwidth]{images/plotting_screenshot.png}
  \end{center}
  \caption{Screenshot of ipython showing plots of the CoM obtained by a simulation}
\end{figure}
