# Dynamic Simulation

To evaluate the generated trajectories a simulator for the dynamics was developed.
The simulator was build on the \name{SimDynamics} framework that is part of \name{Simox}.
\name{SimDynamics} uses \name{Bullet Physics} as underlying physics framework.
A big part of the work on the simulator was spend on configuring the parameters and finding flaws in the physics simulation.
Thus the simulator includes a extensive logging framework that measures all important parameters of the simulation.
For visulizing and analysing the measurement the Open Source tools \name{IPython}, numpy and \name{pandas} where used.

## Simulating rigid body dynamics {#section:rigid-body-simulation}

For physical simulation in general can be devided into discrete methodes and continous methodes.
Discrete simulators only compute the state of the system at specific points in time, while
continous simulators are able to compute the state of the system at any point in time.
While contious simulation is the more flexible approach, it quickly becomes impractical
with the number of constrains involved. Typically a large amount of differential equations
need to be solved. Since it is hard to obtain analytical solutions for most differential equations,
numerical methodes need to be used, which often have a large runtime.
On contrast discrete simulation methodes only compute simulation values for specific time steps.
This exploits the observation that we will typically query the state of the physics engine only
at a fixed rate anyway, e.g. at each iteration of our control loop).
Rather than solving the differntial equations that describe the physical system in each step,
a solution is derived from the previous simulation state.

A physical system we can typically find two kind of forces: Applied forces and constraint forces.
Applied forces are the input forces of the system. Source of applied forces are for example objects like springs or gravity.
Constraint forces are fictious forces that arrise from contrains we impose on the system:
Non-penetration constraints, friction constraints, position constrains of joints or velocity constrains
for motors.
Mathematically we can express such constrains in the form: $C(x) = 0$ or $\dot{C}(x) = 0$ in the case of equality constrains,
or as $C(x) \geq 0$ or $\dot{C(x)} \geq 0$ in the case of inequality constrains.
For example the position constraint of a joint $p$ connected to a base $p_0$ with distance $r_0 = ||p-p_0||$
would be: $C(p) = || p - p_0 ||^2 - r_0^2$
If $p$ is moving with a linear velocity $v$ a constraint force $F_c$ is applied to $p$ to maintain this constraint.
We can view $C$ as a transformation from our cartesian space to the constraint space. Thus by computing the jacobian
$J$ of $C$ we can relate velocities in both spaces. Furthermore we can realte constraint space forces $\lambda$ with cartesian space forces using the transpose of the Jacobian.
Thus if we can find the constraint space force $\lambda$ that is needed to maintain this constraint we can compute $F_c$ using $F_c = J^T \lambda$.
Computing this constraint space forces is the task of the constraint solver.

The constrained solver used by \name{Bullet} and thus the constraint solver used for simulating the
patterns here is a sequential impulse solver.
To make some calculations easier, a SI solver works with impulses and velocities rather than forces and accelerations.
Impulses and forces can be easily transformed in eachother as $P = F \cdot T$ where $P$ is the impulse and $T$ the timestep size.
A sequential impulse solver tries to compute the constraint force (in this case rather impulse) $\lambda$ for each costraint *seperately*.
For each constraint the following steps are executed:

1. Compute the velocity that results from *applied forces* on the body
2. Calculate constraint force to satisfy the velocity constraint
3. Compute new velocity resulting from constraint force *and* applied force on the body
4. Update position of the body by integrating velocity: $p[n+1] = p[n] + v \cdot T$

Of course this might not lead to a global solution, as satisfying a constraint might violate a previously solved one.
The idea is to repeadidetly loop over all constraints, so that a global solution will be reached.
Obviously the quality of this methode relies on how often this loop is executed. Consider the case of a kinematic
chain where a movement of a link always violates at least one constraint. It is clear that this methode needs a lot of iterations
to yield good results in this case.
It becomes even worse in the case of a parallel kinematic that is in contact with the ground, as is the case for a bipedal robot in dual support stance.
Solving a non-penetration constrain on either end, will invalidate the position constraint of the next link.
In turn, the position constraint of each link needs to be updated until the other end of the kinematic chain is reached. If the non-penetration
constraint is violated again for this end, the whole process starts again in reverse direction. This leads to oscillations that need a lot more
iterations to level off to an acceptable level.
Dispite these inaccuracies using enought solver iteration this still yields an overall usable systems. However the velocities still will have
a small random error in each simulation step.
This poses a major problem when trying to measure accelerations, as the random error acauses them to accelerate wildly.
This circumstance needs to be taken into account when dealing with values derived from the acceleration (e.g. the ZMP),
as mean-filters might be neccessary.

## Practical challenges of physics simulation

While walking only the feet of the robot are in contact with the ground. Thus the stability of the whole robots depends on the contact of the feet with the floor. Especially in single support phase that area is very small with regard to the size of the robot.
For that reason the accuracy of ground contatc forces and friction is of utmost importance for the quality of the simulation.
In general three classes of errors need to be elimnated to get a good simulation:

1. Incorrectly configured parameters, such as fictions coefficent and contact thresholds

2. Numerical errors

3. Inherent errors of the methode

As outline in the section about discrete time dynamic simulation, the physics of the system
are formulated as input forces and constrains that need to be solved for the constraint forces.
Since \name{Bullet} uses the iterative apporach described in section \ref{section:rigid-body-simulation}. Thus it is important
to use a sufficient amount of iterations for each simulation step. Another important parameter is the timestep of each simulation step.
Through experimental evaluation a simulation with 2000 solver iterations and a timestep size of 1ms was sufficiently stable.
However since the number of iterations is very high and a lot of timesteps are calculated during the simulation, numeric errors become significant.
That made is neccessary to enable using double precision floating point numbers for the values used during simulation.

To decide which contact constrains are active for which points, \name{Bullet} must solve for object collisions. Depending on the objects
involved different algorithms are used to calculate the contact points. Major gains in accuracy could be observed by replacing
the feet and the floor with simple box shapes, instead using mesh based models.

## Simulating walking patterns

The simulator was designed to load arbitrary motions in the \name{MMM} format and replay them. Additional stabilization algorithms can be applied
depending on additional information provided in the \name{MMM} motions.
\todo{Make sure you can really load vanilla \name{MMM} trajectories without crashing}

Even during simple playback of a trajectory, a number of conisderations due to the dynamics need to be taken into account.
We will outline some of the problems and how they where resolved.

Simply applying the joint values at the given point in time, will lead to large jumps in velocity, acceleration and jerk.
This will cause large oscillations, which in turn result in destabilizing disturbances.
Interpolation between the joint angles of two frames can mitigate this. To implement this cubic splines where used instead of linear interpolation, as they also ensure that the velocity is continous.

Disturbances due to the simulation will cause position errors in the joints.
To fix that PID based motor controllers were added to \name{SimDynamics}. They control the motor velocites to compensate position errors.

Since the motors used by the simulation framework are velocity controlled, their acceleration is not limited.
This is not consistent with real motors, thus limits for velocites and acceleration where introduced to \name{SimDynamics},
that can be configured on a per-joint basis.

An important part of the simulation is the generation of measurements that can saved to be carefully evaluted offline or displayed in the visualization.
For this purpose a modular measuremt component was added to the simulator.
An important design goal was to keep the measurement component as simple to extend and maintain as possible.
Each module meassures a specific set of values and writes them, indexed by the corresponding timestamp, to its log file.
As output format the well know plaintext format CSV was used.
The visualization can query the measurement components directly to get the newest values to be displayed.
For example the ZMP module measures the actual ZMP and also provides an interface to query the trajectory ZMP and the reference ZMP that was provided as input for the pattern generator.
Thus all three values can be displayed in the visualization and easily compared later by analysing the log file.
Since the goal was to keep the component as simple as possible, we use exsistening well known tools for analysing the generated log files.
Some small helper scripts are provied to make it easier to load the data into the time series analysis framework \name{pandas}.
\name{Pandas} interfaces with the popular plotting framework \name{matplotlib} to diplay plots of the data.
\name{IPython} is used to easily run the analysis and display the results in a browser window.
All plots of simulated patterns found in this thesis can be generated automatically for every simulation.

\todo{Screenshot of analysis software}
