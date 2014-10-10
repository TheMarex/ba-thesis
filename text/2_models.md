# Models for humanoid walking

## The Linear Inverted Pendulum Model

\todo{Use different name for CoM, $p$ will be rather used for the ZMP, maybe $c$?}

\todo{picture of 3D-LIPM}

A simple model for describing the dynamics of a bipedal robot during single support phase is the 3D inverted pendulum.
We reduce the body of the robot to a point-mass at the center of mass and replace the support leg
by a mass-less telescopic leg which is fixed at a point on the supporting foot.
Initially this will yield non-linear equations that will be hard to control.
Howevery by constraining the movement of the inverted pendulum to a fixed plane, we can derive a linear dynamic system.
This model called the 3D *linear* inverted pendulum model (short *3D-LIPM*).

### The inverted pendulum

To describe the dynamics of the inverted pendulum we are mainly interested in the effect a given actuator torque has on the movement of the pendulum.

For simplicity we assume that the base of the pendulum is fixed at the origin of the current cartesian coordinate system.
Thus we can describe the position inverted pendulum by a vector $p = (x, y, z)$.
We are going to introduce an appropriate (generalized) coordinate system $q = (\theta_R, \theta_P, r)$ to get an easy description of our actuator torques:
Let $m$ be the mass of the pendulum and $r$ the length of the telescopic leg.
$\theta_P$ and $\theta_R$ describe the corresponding roll and pitch angles of the pose of the pendulum.
\todo{add image with angles here}

Now we need to find a mapping between forces in the cartesian coordinate system and the generalized forces (the actuator torques).
Let $\Phi: \mathbb{R}^3 \longrightarrow \mathbb{R}^3, (\theta_R, \theta_P, r) \mapsto (x, y, z)$ be a function that maps the generalized coordinates to the cartesian coordinates.
Then the jacobian $J_\Phi = \frac{\partial p}{\partial q}$ maps the *generalized velocites* to *cartesian velocites*.
Furthermore we know that the transpose $J_\Phi^T$ maps *cartesian forces* $F = m (\ddot x, \ddot y, \ddot z)$ to *generalized forces* $(\tau_r, \tau_p, f)$.

We write $x$, $y$ and $z$ in terms of our generalized coordinates to compute the corresponding jacobian $J_\Phi$.
From the fact that the $\theta_P$ is the angle between the projection of $p$ onto the $xz$-plane and $p$
and $\theta_R$ the angle between $p$ and the projection onto the $yz$ plane we can derive the following equations \todo{reference paper}:

\begin{equation}
\begin{array}{lcll} \label{eq:lip-xyz}
x & = & r \cdot \sin \theta_P & =: r \cdot s_P\\
y & = & -r \cdot \sin \theta_R & =: -r \cdot s_R \\
z & = & \sqrt{r^2 - x^2 - y^2} = r \cdot \sqrt{1 - s_P^2 - s_R^2} & \\
\end{array}
\end{equation}

From which we can compute the jacobian by partial derivation:

\begin{equation} \label{eq:lip-jacobian}
J = \frac{\partial p}{\partial q} = \left( \begin{array}{rcl}
0 & r \cdot c_P & s_P \\
-r \cdot c_R & 0 & s_P \\
\frac{2 \cdot r \cdot s_P c_P}{\sqrt{1 - s_P^2 - s_R^2}} & \frac{2 \cdot r \cdot s_R c_R}{\sqrt{1 - s_P^2 - s_R^2}} & \sqrt{1 - s_P^2 - s_R^2}\\
\end{array}
\right)
\end{equation}

Using the equation of motion as given by
\begin{equation}
\begin{array}{lcr}
F & = & (J^T)^{-1} \Gamma + f_g \\
m \cdot
\left(\begin{array}{c}
\ddot x \\
\ddot y \\
\ddot z \\
\end{array}\right)
& = & (J^T)^{-1}
\left(\begin{array}{c}
\tau_R \\
\tau_P \\
f \\
\end{array}\right)
+
\left(\begin{array}{c}
0 \\
0 \\
-m \cdot g \\
\end{array}\right) \\
\end{array}
\end{equation}

and equations \ref{eq:lip-jacobian} and \ref{eq:lip-xyz} we can derive the following equations:

\begin{equation} \label{eq:lip-dyn-y}
m(-z\ddot{y} + y\ddot{z}) = \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_R} \cdot \tau_R + m g y
\end{equation}

\begin{equation} \label{eq:lip-dyn-x}
m(z\ddot{x} - x\ddot{z}) = \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_P} \cdot \tau_P + m g x
\end{equation}

Observe that the terms of the left-hand side are not linear. To remove that non-linearity
we are going to use the *linear* inverted pendulum model.

### Linearization

In a man-made environment it is fair to assume that the ground a robot will walk on
can be approximate by a slightly sloped plane. In most cases it can even assumed that there is no slope at all.

The basic assumption in the next section will be that the CoM will have a *constant displacement* with regard to our ground plane.
Thus we can constrain the movement of the CoM to a plane that is parallel to the ground plane.
Note that this assumption is, depending on the walking speed, only approximately true for human walking as shown by Orendurff et. al.
For slow to fast walking ($0.7$ m/s and $1.6$ m/s respectively) the average displacement in $z$-direction was found to be between $2.7cm$ and $4.81$ cm.
While the walking patterns generated based on the LIP-model will guarantee dynamic stability, they might not look natural with regard to human walking.

\todo{cite Orendurff}

We are going to constrain the $z$ coordinate of our inverted pendulum to a plane
with normal vector $(k_x, k_y, -1)$ and $z$-displacement $z_c$:

\begin{equation} \label{eq:lip-z-plane}
z = k_x \cdot x + k_y \cdot y + z_c
\end{equation}

Subsequently the second derivative of $z$ can be described by:

\begin{equation} \label{eq:lip-z-div}
\ddot{z} = k_x \cdot \ddot{x} + k_y \cdot \ddot{y}
\end{equation}

Substituing \ref{eq:lip-z-plane} and \ref{eq:lip-z-div} into the equations \ref{eq:lip-dyn-y} and \ref{eq:lip-dyn-x}
yields the following equations:

\begin{equation}
\ddot{y} = \frac{g}{z_c} y - \frac{k_x}{z_c} (x\ddot{y} - \ddot{x}y) - m z_c \cdot \tau_R \cdot \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_R}
\end{equation}
\begin{equation}
\ddot{x} = \frac{g}{z_c} x + \frac{k_y}{z_c} (x\ddot{y} - \ddot{x}y) + m z_c \cdot \tau_P \cdot \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_P}
\end{equation}

The term $x\ddot{y} - \ddot{x}y$ that is part of both equations is still causing the equations to be non-linear.
To make this equations linear we will assume that our ground plane has no slope, thus $k_x = k_y = 0$ and the non-linear terms will vanish.

Another problem is that the actuator torques $\tau_R$ and $\tau_P$ both have non-linear factors $\frac{\sqrt{1 - s_P^2 - s_R^2}}{c_R}$ and $\frac{\sqrt{1 - s_P^2 - s_R^2}}{c_P}$ respectively. This can be solved by substituting with the following *virtual inputs*:

\begin{equation}
\tau_P \cdot \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_P} = u_P
\end{equation}
\begin{equation}
\tau_R \cdot \frac{\sqrt{1 - s_P^2 - s_R^2}}{c_R} = u_R
\end{equation}

Which yields our final description of the dynamics:
\begin{equation} \label{eq:lip-y}
\ddot{y} = \frac{g}{z_c} y - \frac{u_R}{m z_c}
\end{equation}
\begin{equation} \label{eq:lip-x}
\ddot{x} = \frac{g}{z_c} x + \frac{u_R}{m z_c}
\end{equation}

\todo{include pattern generation just based on 3D-LIPM, I don't understand how they derived the controller}

## The Zero Moment Point

A very popular approach to humanoid walking are schemes based on the Zero Moment Point. One reason for that might be that
it is very simple to describe constrains for dynamic stability using this reference point.
As long as the following condition is met we will have full ground contact of our support foot and thus can realize dynamically stable walking:
*The ZMP is strictly inside the support polygone of the support foot.*

For flat ground contact of our support foot with the floor the ZMP corresponds with the position of the center of pressure (CoP).
Indeed, some author (notably Pratt) prefer to use the term CoP instead of ZMP.

The CoP of an object in contact with the ground can be computed as the sum of all contact points $p_1, \dots, p_n$ weighted by the forces in $z$-direction $f_{1z}, \dots, f_{nz}$ that is applied:

\begin{equation} \label{eq:zmp-definition}
p := \frac{\sum^N_{i=1}p_i f_{iz}}{\sum^N_{i=1} f_{iz}}
\end{equation}

An important fact (and the origin of its name) is that there are no torques around the $x$ and $y$ axis at the ZMP:

\begin{equation}
\tau = \sum^N_{i=1} (p_i - p) \times f_i
\end{equation}

Splitting that up into each component using the definition of the cross product yields:

\begin{equation}
\tau_x = \sum^N_{i=1} (p_{iy} - p_y) f_{iz} - \overbrace{(p_{iz} - p_z)}^{=0} f_{iy}
\end{equation}

\begin{equation}
\tau_y = \sum^N_{i=1} \overbrace{(p_{iz} - p_z)}^{=0} f_{ix} - (p_{ix} - p_x) f_{iz}
\end{equation}

\begin{equation}
\tau_z = \sum^N_{i=1} (p_{ix} - p_x) f_{iy} - (p_{iy} - p_y) f_{ix}
\end{equation}

Since we have flat ground contact, all contact points have the same $z$-coordinate as the ZMP, thus we can simplify $\tau_x$ and $\tau_y$ to:

\begin{equation} \label{eq:zmp-torque-x}
\tau_x = \sum^N_{i=1} (p_{iy} - p_y) f_{iz} = \sum^N_{i=1} (p_{iy} f_{iz}) - (\sum^N_{i=0} f_{iz}) \cdot p_y
\end{equation}

\begin{equation}\label{eq:zmp-torque-y}
\tau_y = \sum^N_{i=1} - (p_{ix} - p_x) f_{iz} = \sum^N_{i=1} - (p_{ix} f_{iz}) + (\sum^N_{i=0} f_{iz}) \cdot p_x
\end{equation}

Furthermore we can use the corresponding components $p_x$ and $p_y$ from the definition of the ZMP \ref{eq:zmp-definition}
and substitude in the equations \ref{eq:zmp-torque-x} and \ref{eq:zmp-torque-y}.

This will yield: $\tau_x = \tau_y = 0$.

Please note that $\tau_z$ will in general not be zero, nonetheless in case of straight walking it is often assumed to be zero as well.

## The table-cart model {#section:table-cart}

The table-cart model is equivalent to the 3D-LIPM model discussed before,
but somewhat more intuitive for computing the resulting ZMP from an CoM motion.
The model consists of an (infinitely) large mass-less table of height $z_c$, while the foot of the table has the shape of the support polygone.
Given a frictionless cart with mass $m$ that moves on the table we can compute the resulting ZMP in the support foot.
Please note that the 3D-dimensional model is equivalent to having two independent tables
with two carts each in the $xz$ and $yz$-plane respectively.
First of all, lets compute the torque $\tau_x$ and $\tau_y$ around the x-axis and y-axis at the ZMP on the support foot.

\begin{equation}
\tau_y = \overbrace{-m g (c_x - p_x)}^{\text{torque due to gravity}} + \overbrace{m \ddot{x} \cdot z_c}^{\text{torque due to acceleration of cart}}
\end{equation}

\begin{equation}
\tau_x = -m g (c_y - p_y) + m \ddot{y} \cdot z_c
\end{equation}

Please note the similarity to the equations \ref{eq:lip-y} and \ref{eq:lip-x}
when assuming the base of the pendulum is located at $p$.
If we now use the property of the ZMP that the torque around the $x$ and $y$-axis is zero,
we can solve for the ZMP position $p$:

\begin{equation} \label{eq:zmp-x}
p_x = c_x - \frac{z_c}{g} \ddot{c_x}
\end{equation}

\begin{equation} \label{eq:zmp-y}
p_y = c_y - \frac{z_c}{g} \ddot{c_y}
\end{equation}

## Multi-Body methode to calculate the ZMP {#section:multi-body-zmp}

Besides the simplified table-cart model, there also exsists an exact methode
to calculate the resulting ZMP from the movement from serveral connected rigid bodies.

Let $c_i$ be the CoM position and $m_i$ the mass of the $i$-th body ($i \in \{1, ..., k\}$). Then the total linear
momentum $\mathcal{P}$ can be calculated by:

\begin{equation}
\mathcal{P} = \sum^k_{j=1} m_j \cdot \dot{c}_j
\end{equation}

If $\omega_i$ the angular momentum and $R_i$ is the rotational part of the reference frame of the $i$-th body and $I_i$ the inertia tensor in that reference frame, the total angular momentum $\mathcal{L}$ can be calculated by:

\begin{equation}
\mathcal{L} = \sum^k_{j=1} c_j \times (m_j \dot{c}_j) + R_j I_j R^T_j \omega_j
\end{equation}

If we denote the total mass of the robot with $M$ and the gravity vector with $g$ we can express the change of linear momentum if a force $f$ is applied to the body as:

\begin{equation} \label{eq:change-lin-momentum}
\dot{\mathcal{P}} = M g + f
\end{equation}

And subsequently the change in angular momentum if a torque $\tau$ is applied:

\begin{equation} \label{eq:change-ang-momentum}
\dot{\mathcal{L}} = c \times Mg + \tau
\end{equation}

To calculate the resulting torque $\tau_{ZMP}$ around the ZMP located at $p$ we can use:

\begin{equation} \label{eq:multi-body-zmp}
\tau_{ZMP} = \tau + (0 - p) \times f = \tau - p \times f
\end{equation}

If solve equation \ref{eq:change-lin-momentum} for $f$ and \ref{eq:change-ang-momentum} for $\tau$ and substitute them in \ref{eq:multi-body-zmp}
this yields the following equation:

\begin{equation}
\tau_{ZMP} = \dot{\mathcal{L}} - c \times M g - p \times (\dot{\mathcal{P}} - Mg)
\end{equation}

Since we know that the torque around the ZMP is zero around the $x$ and $y$ axis
we can apply the definition of the cross product and solve for the ZMP position:

\begin{equation}
p_x = \frac{Mgx + p_z \dot{\mathcal{P}}_x - \dot{\mathcal{L}}_y}{Mg + \dot{\mathcal{P}}_z}
\end{equation}
\begin{equation}
p_y = \frac{Mgy + p_z \dot{\mathcal{P}}_y - \dot{\mathcal{L}}_x}{Mg + \dot{\mathcal{P}}_z}
\end{equation}

Both equations are dependent on $p_z$. If we assume the robot walks on a flat
floor, we can set $p_z = 0$.

See figure \ref{zmp-comparision} to get an idea how much the Multi-Body ZMP
derives from the estimation using the Cart-Table model.

\begin{figure*}[tb]
\includegraphics[width=\textwidth,resolution=300]{images/zmp_comparison.png}
\caption{Comparision of the Cart-Table and Multi-Body to estimate the realized ZMP during walking.}
\label{img:zmp-comparison}
\end{figure*}

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

