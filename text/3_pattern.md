# Pattern generator

To generate a walking pattern for a bipedal robot two basic approaches are common:

1. Generate (or modify) foot trajectories that realize a prescribed trajectory of the CoM

2. Generate a CoM trajectory for prescribed foot trajectories

The first approach is generally used for implementing pattern generators solely based
on the 3D-LIP model. \cite{kajita20013d} 
The second approach is the more versatile one, since it is easy to incorporate constrains
of our environment (e.g. only limited foot holds) in the input of the pattern generator.
However care must be taken while choosing adequate step width and step length parameters for the foot trajectory,
so that they can actually be realized by the robot.
The pattern generator proposed by Kajita et al. \cite{kajita2003biped} based on Preview Control
realizes the second approach.
We will discuss the theoretical background of this pattern generator here in more detail,
since all pattern that we used where generated that way.

## Computing the CoM from a reference ZMP

As we saw in the section \ref{section:table-cart} it is easy to compute the resulting
ZMP given the CoM and its acceleration. However for generating the walking
pattern, we want to compute the CoM trajectory from a given ZMP.
If you rearrange the equations \ref{eq:zmp-x} and \ref{eq:zmp-y} you see that we have to solve a second order differential equations:

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

We use equation \ref{eq:zmp-x} to calculate the actual output of the dynamic system the resulting ZMP, that will be controlled:

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
are constant. We can therefore use the standard approach to solve this system using the equation:

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
It is easy to see that $A_0$ is nilpotent and $A_0^3 = 0$, thus the computation simplifies to the following:

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
reference ZMP trajectory. A performance index $J_x$ for a given control input $u_x$ is needed to formalize what a "good" control input would be.
A naive performance index could be:
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

As you can see the reference ZMP is perfectly tracked. However, the CoM does not behave as expected.
To achieve the required ZMP position the CoM will be *accelerated indefinitely* in the opposite direction.
Clearly this is not desired and will lead to falling on a real robot. A more sophisticated performance index is needed.
To eventually reach a stable state at which the CoM comes to rest, the performance index should include a state feedback.
Also note the large jerk that is applied to the system when the reference ZMP position changes rapidly.
In a real mechanical system large jerks will lead to oscillations, which will disturb the system.
Thus the performance index should also try to limit the applied jerk.

Another problem is caused by the very nature of a controller:
The controller starts to act *after* we have a deviation from our reference ZMP trajectory.
Trying make this lag as small as possible can lead to very high velocities, which might not be realizable by motors of a robot.
However we have at least limited knowledge of the future reference trajectory. This knowledge can be leveraged
by using Preview Control, which considers the next $N$ time steps for computing the performance index.

Kajita et. al. use a performance index proposed by Katayama et. al. \cite{katayama1985design} to solve all of the problems above:

\begin{equation}
J_x[k] = \sum^{\infty}_{i=k} Q_e e[i]^2 + \Delta x[i]^T Q_x \Delta x[i] + R \Delta u_x[i]^2
\end{equation}

$Q_e$ is the error gain, $Q_x$ a symmetric non-negative definite matrix (typically just a diagonal matrix)
to weight the components of $\Delta x[i]$ differently and $R > 0$.
Conveniently Katayama also derived an optimal controller for this performance index, which is given by:

\begin{equation}
u[k] = -G_i \sum^k_{i=0} e[k] - G_x x[k] - \sum^N_{j=1} G_p p^{ref}_x[k + j]
\end{equation}

The gains $G_i, G_x, G_p$, can be derived from the parameters of the performance index.
Since the calculation is quite elaborate we refer to the cited article by Katayama p. 680 for more details.

## Implementation

\begin{figure*}[tb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/pattern_generator_architechture.png}
\caption{Architechture of the pattern generator}
\label{img:pattern-generator-architechture}
\end{figure*}

To generate walking patterns based on the ZMP preview control method, the approach from Kajita
was implemented in \name{libBipedal} a shared library. A front-end was developed to easily change parameters, visualize
 and subsequently export the trajectory to the \name{MMM} format.
The implementation was build on a previous implementation, which was refactored,
extended and tuned with respect to results from the dynamics simulation.

The pattern generator makes extensive usage of \name{Simox VirtualRobot}, for providing a model of the robot
and the associated task of computing the forward- and inverse kinematics.

Generating a walking pattern consists of multiple steps. First the foot positions are calculated. These are used to derive the reference ZMP
trajectory which is feed into the ZMP preview controller. From that the CoM trajectory is computed. The CoM trajectory and feet trajectories are
then used to compute the inverse kinematics. The resulting joint trajectory is displayed in the visual front-end and can be exported.
Each step is contained in dedicated modules that can be easily replaced, if needed.
We will outline the implementation of each module separately.

### Generating foot trajectories

To generate the foot trajectories several parameters are needed:

\begin{figure*}[b]
\begin{center}
  \begin{tabular}{| l | l |}
    \hline
    $h$ & 0.1 m \\ \hline
    $l$ & 0.3 m \\ \hline
    $w$ & 0.2 m \\ \hline
    $t_{ss}$ & 0.7 s\\ \hline
    $t_{ds}$ & 0.1 s\\ \hline
  \end{tabular}
\caption{Default parameters used for generating a walking trajectory.}
\label{table:pattern-parameters}
\end{center}
\end{figure*}

Step height $h$
  ~  Maximum distance between the foot sole and the floor

Step length $l$
  ~ Distance in anterior direction ($y$-Axis) between the lift-off point and the touch-down point

Step width $w$
  ~ Distance in lateral direction ($x$-Axis) between both TCP on the feet

Single support duration $t_{ss}$
  ~ Time the weight of the robot is only support by exactly one foot

Dual support duration $t_{ds}$
  ~ Time the weight of the robot is supported by both feet

See figure \ref{table:pattern-generator} for the values used to generate the trajectories
using a model of the \name{Armar IV} robot.

#### Walking straight

Since the foot trajectories of a humanoid walking have a cyclic nature, we only need three different foot trajectories that can be composed
to arbitrarily long trajectories:
Two transient trajectories for the first and last step respectively and a cyclic motion that can be repeated indefinitely.
We can use the same trajectories for both feet, as they are geometrically identical.
Each foot trajectory starts with swing phase and a resting phase. The trajectory in $y$ and $z$ direction is computed by a 5th order polynomial
that assures the velocities and accelerations are approaching zero at the lift-off and touch-down point.
The first and last step only have half of the normal step length, since the trajectory is starting and ending
from a dual support stance, where both feet are placed parallel to each other.
Each trajectory is encoded as a $6 \times N$ matrix, each column containing Cartesian coordinates and roll, pitch and yaw angles.

#### Walking on a circle

Much of the general structure of the foot trajectory remains the same as for walking straight.
However instead of specifying the step length, it is implicitly given by the segment of the circle that should be traversed and the number of steps.
So extra care needs to be taken to specify enough steps so that the generated foot positions are still.
Each foot needs to move on a circle with radius $r_{inner} = r - \frac{w}{2}$ or $r_{outer} = r + \frac{w}{2}$ depending which foot lies in the direction of the turn. The movement in $z$-direction remains unaffected. However the movement in the $xy$-plane is transformed to follow the circle for the specific foot.
\todo{Current implementation does effectively that, but is actually a hack. Needs separate trajectories for left/right}
The same polynomial that was previously used for the $y$-direction is now used to compute the
angle on the corresponding circle and the $x$ and $y$ coordinates are calculated accordingly.
The foot orientation is computed from the tangential (y-Axis) and normal (x-Axis) of circle the foot follows.

#### Balancing on one foot

To test push recovery from single support stance a special pattern was needed. To generate this
another footstep planer was implemented that generates a trajectory for standing on one foot.
Starting from dual support stance, the swing leg is moved in vertical direction until the usual step height is achieved.
Additionally the foot is moved in lateral direction to half the step width. This reduces the necessary upper body tilt to compensate the imbalance.
For the last step the inverse movement is performed to get back into dual support stance.
This method could be extended to walk by setting the next support foot in a straight line before the current support foot.
The swing foot would need to be moved in an arc in lateral direction to avoid self-collisions.


### ZMP reference generation

As an input for the ZMP preview control, we need a reference ZMP movement that corresponds with the foot trajectory.
The reference generator receives a list of intervals associated with the desired support stance and foot positions as input.
In single support phase, the reference generator places the ZMP in the center of the support polygon of the corresponding foot.
Since the support polygon is convex, the center is the point furthest away from the border of the polygon. Thus it should guarantee a maximum
of stability with regard to possible ZMP errors.
In dual support phase, the reference generator shifts the ZMP from the previous support foot to the next support foot.
Kajita et. al. suggest using a polynomial to interpolate the ZMP positions between the feet.
However a simple step function
$\sigma(t) = \left\{\begin{array}{lr}p_1 & t \leq t_0 \\ p_2 & t > t_0 \end{array}\right.$
seems to suffice as well.
Since the touch-down of the swing foot might have a small lag, it is important that $t_0$ is the middle of the dual support phase.
This assures we do not start to move the ZMP too early.

### ZMP Preview Control

This module implements the method described by Kajita et. al. and uses the method outlined by Katayama et. al. to compute the optimal control
input $u[k]$. Since it is computational feasible, the preview period consists of the full reference trajectory.
For an online usage of this method, this could be reduced to a much smaller sample size, e.g. only preview one step ahead.
Using the system dynamics described by \ref{eq:state-transition-result} the CoM trajectory, velocity and acceleration can be computed.
The implementation makes heavy use of \name{Eigen}, a high performance linear algebra framework that uses SIMD instructions to speed up calculations.
Thus thus a calculation time of 6.2s could be achieved to calculate ten steps, including the inverse kinematics.

### Inverse Kinematics

Using the foot trajectories and CoM trajectory the actual resulting joint angles need to be calculated.
Since the kinematic model that is used has a total of 35 degrees of freedom, we need to reduce the number of joints that are used to
a sensible value.
For walking only the joints of the legs and both the torso roll and pitch joints are used. All other joints are constrained to static values that will not cause self-collisions (e.g. the arms are slightly extended and do not touch the body).
For computing the IK additional constraints where added, to make sure the robot has a sensible pose: The chest should always have an upright position
and the pelvis should always be parallel to the floor. To support non-straight walking, the pelvis and chest orientation should also follow the walking direction. Thus the following method to compute the desired chest and pelvis orientation is used:

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

A hierarchical solver was used to solve the inverse kinematics for that goals in the given order.
It was observed that specifying a good target height for the CoM is of utmost importance for the quality of the IK.
For the model of \name{Armar IV} that used here, a height of $0.86$ m yielded the best results.

### Trajectory Export

The trajectory was exported in open \name{MMM} trajectory format. The format was extended to export additional information
useful for debugging and controlling the generated trajectory.
That means besides the joint values and velocities the trajectory also includes the CoM and ZMP trajectory that was used to derive them.
Also information about the current support phase is saved.
For convenience the pose of chest, pelvis, left and right foot are exported as homogeneous matrices as well.
This was done to save the additional step of computing them again from the exported joint trajectory for the stabilizer
and also eliminate an additional error source while debugging.


