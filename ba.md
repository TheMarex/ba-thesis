# Introduction
motivation,  and a bit of overview of humanoid walking.  I recommend to leave it for later, start with the sections that you feel its easier to write (usually, the ones that have more content).

- motivation:
	- navigating in human environments
- walking in humans:
	- CoM movement, gait phases, differences to what we do here
- static vs. dynamic walking
- overview of models used for dynamic walking

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

\todo{Maybe describe the full-body methode to compute the ZMP}

## Simulating rigid body dynamics

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

# Pattern generator

To generate a walking pattern for a bipedal robot two basic approaches are common:

1. Generate (or modify) foot trajectories that realize a prescribed trajectory of the CoM

2. Generate a CoM trajectory for prescribed foot trajectories

The first approach is generally used for implementing pattern generators soley based
on the 3D-LIPM model. \todo{citation needed}

The second approach is the more versatile one, since it is easy to incorporate constrains
of our environment (e.g. only limited foot holds) in the input of the pattern generator.
However care must be taken while chosing adequate step width and step length parameters for the foot trajectory,
so that they can actually be realized by the robot.

The pattern generator proposed by Kajita et al. \todo{add citation} based on Preview Control
realizes the second approach.
We will discuss the theoretical background of this pattern generator here in more detail,
since all pattern that we used where generated that way.

## Computing the CoM from a reference ZMP

As we saw in the section \ref{section:table-cart} it is easy to compute the resulting
ZMP given the CoM and its acceleration. However for generating the walking
pattern, we want to compute the CoM trajectory from a given ZMP.
If you rearange the equations \ref{eq:zmp-x} and \ref{eq:zmp-y} you see that we have to solve a second order differential equations:

\begin{equation} \label{eq:com-x}
c_x = \frac{z_c}{g} \cdot \ddot{c_x} + p_x
\end{equation}

\begin{equation} \label{eq:com-y}
c_y = \frac{z_c}{g} \cdot \ddot{c_y} + p_y
\end{equation}

There are several ways to solve this differential equations, for example by transforming
them to the frequency-domain. This however would mean, the ZMP trajectory needs to be transformed
to the frequency domain as well, e.g. using Fast Fourier Transformation. This has two main
problems:

1. It has a significant computational overhead. (For FFT the additional runtime would be in $O(n \log n)$)

2. We need to know the whole ZMP trajectory in advance.

Instead Kajita et. al. chose to define a dynamic system in the time domain that describes the CoM movement.

### Pattern generation as dynamic system
\todo{maybe do a formal introduction into dynamic system and the state space approach}

For simplicity we will only focus on the dynamic description of one dimension, as the other one is analogous.
To transform the equations to a strictly proper dynamical system, we need to determine the state vector of our system.
For the table-cart model it suffices to know the position, velocity and acceleration of the cart.
Thus the state-vector is defined as $x = (c_x, \dot{c_x}, \ddot{c_x})$. We can define the evolution of the state vector as follows:

\begin{equation} \label{eq:dyn-system}
\frac{d}{dt} \left(\begin{array}{c}
c_x \\
\dot{c_x} \\
\ddot{c_x} \\
\end{array} \right)
=
\overbrace{
\left(\begin{array}{ccc}
0 & 1 & 0\\
0 & 0 & 1 \\
0 & 0 & 0 \\
\end{array}\right)
}^{ =: A_0}
\cdot
\left(\begin{array}{c}
c_x \\
\dot{c_x} \\
\ddot{c_x} \\
\end{array}\right)
+
\overbrace{
\left(\begin{array}{ccc}
0 & 0 & 0\\
0 & 0 & 0\\
1 & 0 & 0\\
\end{array}\right)
}^{ =: B_0}
u
\end{equation}

As you can see the jerk of the CoM was introduced as an input $u_x = \frac{d}{dt} \ddot{c_x}$ into the dynamic system.

We use equation \ref{eq:zmp-x} to calculate the actual output of the dynamic system the resulting zmp, that will be controlled:

\begin{equation} \label{eq:zmp-x-output}
p_x =
\left(\begin{array}{ccc}
1 & 0 & \frac{-z_c}{g} \\
\end{array}\right)
\cdot
\left(\begin{array}{c}
c_x \\
\dot{c_x} \\
\ddot{c_x} \\
\end{array}\right)
\end{equation}

Using this formulation of the dynamic system we need to derive the evolution of our state vector using the state-transition matrix.
Since our input ZMP trajectory will consist of discrete samples at equal time intervals $T$
we define the discrete state as $x[k] := x(k \cdot T)$.
Please note that this system is a linear time-invariant system (LTI), and both matrices $A_0$ and $B_0$
are constant. We can therefore use the standart approach to solve this system using the equation:

\begin{equation}
x(t) = e^{A_0 \cdot (t - \tau)} x(\tau) + \int^t_\tau e^{A_0 \cdot (t - \lambda)} B_0 u(\lambda) d\lambda
\end{equation}

In our discrete case that becomes:
\begin{eqnarray} \label{eq:state-transition-discrete}
x[k+1] & = & e^{A_0 \cdot ((k+1)T - kT)} x[k] + \int^{(k+1)T}_{kT} e^{A_0 \cdot ((k+1)T - \lambda)} B_0 u(\lambda) d\lambda \\
       & = & e^{A_0 \cdot T} x[k] + \left(\int^{(k+1)T}_{kT} e^{A_0 \cdot ((k+1)T - \lambda)} d\lambda \right) \cdot B_0 u[k]\\
       & = & e^{A_0 \cdot T} x[k] + \left(\int^{0}_{T} e^{A_0 \cdot \lambda} d\lambda\right) \cdot B_0 u[k]
\end{eqnarray}
Keep in mind that $u(\lambda) = u[k], \lambda \in (kT, (k+1)T)$ so we can move it outside of the integral.
Let us first compute a general solution for the matrix exponential $e^{A_0 \cdot t}$.
It is easy to see that $A_0$ is nilpotent and $A_0^3 = 0$, thus the computation simpilfies to the following:

\begin{equation}
e^{A_0 t} := \sum^{\infty}_{i=0} \frac{(A_0 \cdot t)^i}{i!} = I + A_0 \cdot t + A_0^2 \cdot \frac{t^2}{2} + 0
=
\left(\begin{array}{ccc}
1 & t & \frac{t^2}{2}\\
0 & 1 & t \\
0 & 0 & 1 \\
\end{array}\right)
\end{equation}

Using this solution computing the integral in \ref{eq:state-transition-discrete} is quite easy:
\begin{equation}
\int^{0}_{T} e^{A_0 \cdot \lambda} d\lambda =  -\int^{T}_{0} \left(\begin{array}{ccc}1 & t & \frac{t^2}{2}\\0 & 1 & t\\ 0 & 0 & 1\end{array}\right) dt
                                            =  -\left.\left(\begin{array}{ccc} %
                                                          t & \frac{t^2}{2} & \frac{t^3}{6} \\ %
                                                          0 & t             & \frac{t^2}{2} \\ %
                                                          0 & 0             & t %
                                                 \end{array}\right)\right|_{0}^{T}\\
                                            =   \left(\begin{array}{ccc} %
                                                          T & \frac{t^2}{2} & \frac{T^3}{6} \\ %
                                                          0 & T             & \frac{T^2}{2} \\ %
                                                          0 & 0             & T %
                                                \end{array}\right)
\end{equation}

Substituting the results in \ref{eq:state-transition-discrete} yields:

\begin{eqnarray} \label{eq:state-transition-result}
x[k+1] & = &  \overbrace{\left(\begin{array}{ccc} %
                     T & \frac{t^2}{2} & \frac{T^3}{6} \\ %
                     0 & T             & \frac{T^2}{2} \\ %
                     0 & 0             & T %
               \end{array}\right)}^{=: A} x[k]
             + \overbrace{\left(\begin{array}{ccc} %
                      \frac{T^3}{6} \\ %
                      \frac{T^2}{2} \\ %
                      T %
               \end{array}\right)}^{=: B} \cdot u_x[k]
\end{eqnarray}

### Controlling the dynamic system

To control this dynamic system we need to determine an adequate control input $u_x$ to realize the
reference ZMP trajectory. A performence index $J_x$ for a given control input $u_x$ is needed to formalize what a "good" control input would be.
A naive performence index could be:
\begin{equation}
J_x[k+1] := (p^{ref}_x[k+1] - p_x[k+1])^2
\end{equation}
To minimize it, we need to find $u_x$ for which $p_x = p^{ref}_x$.
By substituting $p_x[k+1]$ with \ref{eq:zmp-x-output} and $x[k+1]$ with \ref{eq:state-transition-result} this yields:

\begin{equation}
u_x[k] = \frac{p^{ref}_x[k+1] - C \cdot A \cdot x[k]}{C \cdot B} = \frac{p^{ref}_x[k+1] - (1, T, \frac{1}{2} T^2 -\frac{z_c}{g}) \cdot x[k]}{\frac{1}{6}T^3 - \frac{z_c}{g} T}
= \frac{p^{ref}_x[k+1] - p_x[k] - T \dot{c_x}[k] - \frac{1}{2} T^2 \ddot{c_x}[k]}{\frac{1}{6}T^3 - \frac{z_c}{g} T}
\end{equation}

To analyse the behaviour of this control law for $u_x$ we simulate the rapid change of reference ZMP when changing the support
foot. \todo{insert plot}

As you can see the reference ZMP is perfectly tracked. However, the CoM does not behave as expected. To achive the required ZMP position the CoM will be *accelerated indefinietly* in the opposite direction.
Clearly this is not desired and will lead to falling on a real robot. A more sophisticated performence index is needed.
To eventually reach a stable state at which the CoM comes to rest, the performence index should include a state feedback.
Also note the large jerk that is applied to the system when the reference ZMP position changes rappidly.
In a real mechanical system large jerks will lead to oszillations, which will disturbe the system.
Thus the performence index should also try to limit the applied jerk.

Another problem is caused by the very nature of a controller:
The controller starts to act *after* we have a deviation from our reference ZMP trajectory.
Trying make this lag as small as possible can lead to very high velocities, which might not be realizable by motors of a robot.
However we have at least limited knowledge of the future reference trajectory. This knowledge can be leveraged
by using Preview Control, which considers the next $N$ timesteps for computing the performence index.

\todo{add citation katayama}
Kajita et. al. use a performence index proposed by Katayama et. al. to solve all of the problems above:

\begin{equation}
J_x[k] = \sum^{\infty}_{i=k} Q_e e[i]^2 + \Delta x[i]^T Q_x \Delta x[i] + R \Delta u_x[i]^2
\end{equation}

$Q_e$ is the error gain, $Q_x$ a symmetric non-negative definite matrix (typically just a diagonal matrix)
to weight the components of $\Delta x[i]$ differently and $R > 0$.
Conveniently Katayama also derived an optimal controller for this performence index, which is given by:

\begin{equation}
u[k] = -G_i \sum^k_{i=0} e[k] - G_x x[k] - \sum^N_{j=1} G_p p^{ref}_x[k + j]
\end{equation}

The gains $G_i, G_x, G_p$, can be derived from the parameters of the performence index.
Since the calculation is quite elaborate we refer to the cited article by Katayama p. 680 for more details.

## Implementation

\todo{block diagramm of architechture}
To generate walking patterns based on the ZMP preview control methode, the approach from Kajita
was implemented in a shared library. A front-end was developed to easily change parameters, visualize
 and subsequently export the trajectory to the \name{MMM} format.
The implementation was build on a previous implementation, which was refactored,
extended and tuned with respect to results from the dynamics simulation.

The pattern generator makes extensive usage of \name{Simox VirtualRobot}, for providing a model of the robot
and the associated task of computing the forward- and inverse kinematics.

Generating a walking pattern consists of multiple steps. First the foot positions are calculated. These are used to derive the reference ZMP
trajectory which is feed into the zmp preview controller. From that the CoM trajectory is computed. The CoM trajectory and feet trajectories are
then used to compute the inverse kinematics. The resulting joint trajectory is displayed in the visual front-end and can be exported.
Each step is contained in dedicated modules that can be easily replaced, if needed.
We will outline the implementation of each module seperately.

### Generating foot trajectories

To generate the foot trajectories several parameters are needed:

\todo{table with used parameters}

Step height $h$
  ~  Maximum distance between the foot sole and the floor

Step length $l$
  ~ Distance in anterior direction ($y$-Axis) between the lift-off point and the touch-down point

Step width $w$
  ~ Distance in lateral directoin ($x$-Axis) between both TCP on the feet

Single support duration $t_{ss}$
  ~ Time the weight of the robot is only support by exactly one foot

Dual support duratoin $t_{ds}$
  ~ Time the weight of the robot is supported by both feet

#### Walking straight

Since the foot trajectories of a humanoid walking have a cyclic nature, we only need three different foot trajectories that can be composed
to arbitrarily long trajectories:
Two transient trajectories for the first and last step respectively and a cyclic motion that can be repeated indefinetly.
We can use the same trajectories for both feet, as they are geometrically identical.
Each foot trajectory starts with swing phase and a resting phase. The trajectory in $y$ and $z$ direction is computed by a 5th order polynomial
that assures the velocities and accelerations are approaching zero at the lift-off and touch-down point.
The first and last step only have half of the normal step length, since the trajectory is starting and ending
from a dual support stance, where both feet are placed parallel to eachother.
Each trajectory is encoded as a $6 \times N$ matrix, each column containing cartesian coordinates and roll, pitch and yaw angles.

#### Walking on a circle

Much of the general structure of the foot trajectory remains the same as for walking straight.
However instead of specifing the step length, it is implicitly given by the segment of the cricle that should be traversed and the number of steps.
So extra care needs to be taken to specify enough steps so that the generated foot positions are still.
Each foot needs to move on a circle with radius $r_{inner} = r - \frac{w}{2}$ or $r_{outer} = r + \frac{w}{2}$ depending which foot lies in the direction of the turn. The movement in $z$-direction remains unaffected. However the movement in the $xy$-plane is transformed to follow the circle for the specific foot.
\todo{Current implementation does effectively that, but is actually a hack. Needs seperate trajectories for left/right}
The same polynomial that was previously used for the $y$-direction is now used to compute the angle on the corresponding circle and the $x$ and $y$ coordinates are calculated acordingly.
The foot orientation is computed from the tangential (y-Axis) and normal (x-Axis) of circle the foot follows.

#### Balancing on one foot

To test push recovery from single support stance a special pattern was needed. To generate this
another footstep planer was implemented that generates a trajectory for standing on one foot.
Starting from dual support stance, the swing leg is moved in vertical direction until the usual step height is achieved.
Additionally the foot is moved in lateral direction to half the step width. This reduces the necessary upper body tilt to compensate the inbalance.
For the last step the inverse movement is performed to get back into dual support stance.
This methode could be extended to walk by setting the next support foot in a straight line before the current support foot.
The swing foot would need to be moved in an arc in lateral direction to avoid self-collisions.
\todo{It is easy to extent this: DO IT.}


### ZMP reference generation

As an input for the ZMP preview control, we need a reference ZMP movement that corresponds with the foot trajectory.
The reference generator receives a list of intervals associated with the desired support stance and foot positions as input.
In single support phase, the reference generator places the ZMP in the center of the support polygone of the corresponding foot.
Since the support polygone is convex, the center is the point furthest away from the border of the polygone. Thus it should guarantee a maxium
of stability with regard to possible ZMP errors.
In dual support phase, the reference generator shifts the ZMP from the previous support foot to the next support foot.
Kajita et. al. suggest using a poylnomial to interpolate the ZMP positions between the feet.
However a simple step function
$\sigma(t) = \left\{\begin{array}{lr}p_1 & t \leq t_0 \\ p_2 & t > t_0 \end{array}\right.$
seems to suffice as well.
Since the the touch-down of the swing foot might have a small lag, it is important that $t_0$ is the middle of the dual support phase.
This assures we do not start to move the ZMP too early.

### ZMP Preview Control

This module implements the methode described by Kajita et. al. and uses the methode outlined by Katayama et. al. to compute the optimal control
input $u[k]$. Since it is computational feasable, the preview periode consists of the full reference trajectory.
For an online usage of this methode, this could be reduced to a much smaller sample size.
\todo{Add timings, implement configurable preview periode}
Using the system dynamics described by \ref{eq:state-transition-result} the CoM trajectory, velocity and acceleration can be computed.
The implementation makes heavy use of Eigen, a high performence linear algebra framework that uses SIMD instructions to speed up calculations.
Thus thus a calculation time of \fixme{calculation time} could be achieved.

### Inverse Kinematics

Using the foot trajectories and CoM trajectory the actual resulting joint angles need to be calculated.
Since the kinematic model that is used has a total of \fixme{DOF} degrees of freedom, we need to reduce the number of joints that are used to
a sensible value.
For walking only the joints of the legs and both the torso roll and pitch joints are used. All other joints are constrained to static values that will not cause self-collisions (e.g. the arms are slightly extended and do not touch the body).
For comptuing the IK additional constraints where added, to make sure the robot has a sensible pose: The chest should always have an upright position
and the pelvis should always be parallel to the floor. To support non-straight walking, the pelvis and chest orientation should also follow the walking direction. Thus the following methode to compute the desired chest and pelvis orientation is used:

1. Compute walking direction $y'$ as normed mean of y-Axis of both feet: $y' := \frac{y_{left} + y_{right}}{|y_{left} + y_{right}|}$

2. Both should have an upright position $z' := (0, 0, 1)^T$

3. Compute $x'$ as the normal to both vectors: $x' := y' \times z'$

4. Pose $R'$ is given by $R' = (x', y', z')$

A special property of the model that was used for computing the inverse kinematics, is that TCP of the left leg was chosen as root node.
Since we can specify the root position freely, that removes the need of solving for the left foot pose. Thus the following goals need to be satisfied by the inverse kinematics:

1. Chest orientation

2. Pelvis orientation

3. CoM position

4. Right foot pose

To solve the inverse kinematics a hiearchical solver was used to solve for that goals in the given order. It was observed that specifing a good target height for the CoM is of utmost importance for the quality of the IK. Specifing the CoM height too height or too low can lead to the effective loss of degrees of freedom.

\todo{Maybe a more theoretical explaination?}

### Trajectory Export

The trajectory was exported in open \name{MMM} trajectory format. The format was extended to export additional information
useful for debugging and controlling the generated trajectory.
That means besides the joint values and velocites the trajectory also includes the CoM and ZMP trajectory that was used to derive them.
Also information about the current support phase is saved.
For convinience the pose of chest, pelvis, left and right foot are exported as homogenous matrices as well. This was done to save the additional step of computing them again from the exported joint trajectory for the stabilizer and also reduce an additional error source.

\todo{Maybe doing the FK now would be better and more versatile, since we could feed normal \name{MMM} trajectories in the stabilizer}

## Dynamic simulation

To evaluate the generated trajectories a simulator for the dynamics was developed.
The simulator was build on the \name{SimDynamics} framework that is part of \name{Simox}.
\name{SimDynamics} uses Bullet Physics as underlying physics framework.
A big part of the work on the simulator was spend on configuring the parameters and finding flaws in the physics simulation.
Thus the simulator includes a extensive logging framework that measures all important parameters of the simulation.
For visulizing and analysing the measurement the Open Source tools \name{IPython}, numpy and \name{pandas} where used.

### Practical challenges of physics simulation

While walking only the feet of the robot are in contact with the ground. Thus the stability of the whole robots depends on the contact of the feet with the floor. Especially in single support phase that area is very small with regard to the size of the robot.
For that reason the accuracy of ground contatc forces and friction is of utmost importance for the quality of the simulation.
In general three classes of errors need to be elimnated to get a good simulation:

1. Incorrectly configured parameters, such as fictions coefficent and contact thresholds

2. Numerical errors

3. Inherent errors of the methode

As outline in the section about discrete time dynamic simulation, the physics of the system
are formulated as input forces and constrains that need to be solved for the constraint forces.
Since bullet uses an iterative approach that solves each constraint independently, it is of utmost importance
to use a sufficient amount of iterations for each simulation step. Another important parameter is the timestep of each simulation step.
Through experimental evaluation a simulation with 2000 solver iterations and a timestep size of 1ms was sufficiently stable.
However since the number of iterations is very high and a lot of timesteps are calculated during the simulation, numeric errors become significant.
That made is neccessary to enable using double precision floating point numbers for the values used during simulation.

To decide which contact constrains are active for which points, Bullet must solve for object collisions. Depending on the objects
involved different algorithms are used to calculate the contact points. Major gains in accuracy could be observed by replacing
the feet and the floor with simple box shapes, instead using mesh based models.

### Simulating walking patterns

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
All plots of simulated patterns found in this thesis can be generated automatically for very simulation.

\todo{Screenshot of analysis software}

# Stabilizing a trajectory

While executing a trajectory there are several sources of errors that will make it neccessary to correct the trajectory.
We can devide them in about three main classes:

Disturbances of the environment:
  ~ Pattern generator make some assumptions about the environment they operate in.
    Most prominently the 3D-LIMP assumes the floor is completely flat and has no slope.
    Also we assume we can navigate without colliding with other object.
    Any environment that deviates from this assumption can be seen as a disturbance.

Disturbances due to simulation errors:
  ~ Physical simulations often make a tradeoff between speed and simulation accuracy.
    Thus the simulation might not always behave as it was modeled during calculating the pattern, or as it would behave in reality.

Disturbances due to errors of the methode:
  ~ Often pattern generators use simplified models of the dynamics involved to derive generation scheme.
    For example the pattern generator that was used here assumes the ZMP behaves as the cart-table-model predicts.
    However the real ZMP calculated from the multi-body dynamics can substantially deviate.

## Detecting a deviation

When using a ZMP based control scheme to derive a walking pattern it seems natural to check for deviations
of the actual ZMP from the goal ZMP. However a deviation from the reference ZMP does not neccessarily mean we will see any disturbance.
As long a the ZMP remains inside the support polygone the trajectory can be executed as planed.
Also we saw before, it is entirely possible to realize the reference ZMP while being in an overall state that deviates significantly from the
state we assumed while generating the pattern.
Thus it seems more viable to check for a diviation in the trajectory of our CoM. A common approach to correct for CoM position
is to control the pose of the chest frame of the robot. This only works if the majority of the mass of a robot is located in the upper body and arms.
Luckily for most humanoid robots this is the case.

## Stabilizer

Theory
Implementation
Evaluation

# Push recovery
Theory
Implementation
Evaluation

# Results
Make sure that somewhere, either here or in the evaluation sections, you show how you plotted the desired vs. real zmp, even the phantom robot if you want, and the graphics that you generated.

# Conclusions
things to improve
summary of work done and results
