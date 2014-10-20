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

## Controlling a deviation

When using a ZMP based control scheme to derive a walking pattern it seems natural to check for deviations
of the actual ZMP from the goal ZMP. However a deviation from the reference ZMP does not neccessarily mean we will see any disturbance.
As long a the ZMP remains inside the support polygone the trajectory can be executed as planed.
Also we saw before, it is entirely possible to realize the reference ZMP while being in an overall state that deviates significantly from the
state we assumed while generating the pattern.
Thus we also need to check for a diviation in the trajectory of our CoM. A common approach to correct for CoM position
is to control the pose of the chest frame of the robot. This only works if the majority of the mass of a robot is located in the upper body and arms.
Luckily for most humanoid robots this is the case.

## Stabilizer {#section:stabilizer}

We chose a stabilizer proposed by Kajita et. al. in their 2010 paper. \cite{kajita2010biped}
The stabilizer only needs the joint trajectory of the walking pattern augmented with a desired ZMP trajectory.
This allows the stabilizer to use patterns that where generated synthetically, e.g. by a pattern generator, or patterns that are
the results of (adapted) motion capturing.
The methode proposed by Kajita does not need a torque controlled robot, but works with position control.
This was very important for the selection of this stabilizer. The underlying simulation engine \name{Bullet} only supports velocity controlled motors,
consequently the torque can not be controlled directly directly.

The controller works by attaching control frames to specific points on the robot.
The reference position of this frames can be calculated from the input trajectory using forwards kinematics.
To compensate a disturbance the orientation of a reference frame is modified.
The modified reference frames are then converted to the modified joint angles by the inverse kinematics.

\begin{figure*}[tb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/stabilizer_architechture.png}
\caption{Architechture of the stabilizer}
\label{img:archtitechture-stabiluzer}
\end{figure*}

In the remainder of this chapter we will use the superscript $d$ to denote reference values and the subscript $*$
to denote modified values.

For this approach four control frames where selected. The chest to modify the body posture,
the feet to modify the ankle torque, and the pelvis to modify the difference between contact forces of the two feet.

### Reference coordinate system

\begin{wrapfigure}{R}{0.4\textwidth}
\vspace*{-1em}
\includegraphics[width=0.4\textwidth]{images/ground_frame.png}
\caption{Ground frame in dual support}
\label{img:ground-frame}
\end{wrapfigure}

To control the ZMP and CoM it is desireable to have a reference system that is static with respect to the ground in each support phase.
It is convinied to place the reference coordinate system in the center of the respective support polygone.
That means we place the ground frame at the TCP of the respective foot in single support phase.
As the ground frame should be aligned with the floor, we use the projection of the foot poses
to the floor plane.
In dual support phase, we calculate the pose from the position by $p = \frac{1}{2} \cdot (p_{left} + p_{right})$ and
the $y$-axis by $y = \frac{1}{2} \cdot (y_{left} + y_{right})$. The $z$-axis is the normal of the floor plane: $z = (0, 0, 1)$.
See figure \ref{img:ground-frame} for an example.
The resulting reference frame is called the *ground frame*.

### Controlling the body posture

The control strategy of the chest pose is straight forward: Given the reference roll angle $\phi^d$ and reference pitch angle $\theta^d$
compute the differences to the actual angles $\phi$ and $\theta$.
The main problem in a real robot is to obtain the actual global pose of this frame.
The proposed methode is to use a Kalman filter to estimate the pose from the joint position and accelerometers.
We did not implement this methode in simulation, as it is easy to obtain the exact pose from the simulator.
To prevent rapid movements of the chest that cause large accelerations, a dampening controller is used.
The angles $\Delta \phi$ and $\Delta \theta$ can be calculated by the following equations:

\begin{equation}
\Delta \dot{\phi} = \frac{1}{D_c} (\phi^d - \phi) - \frac{1}{T_c} \cdot \Delta \phi
\end{equation}

\begin{equation}
\Delta \dot{\theta} = \frac{1}{D_c} (\theta^d - \theta) - \frac{1}{T_c} \cdot \Delta \theta
\end{equation}

$D_c$ describes the dampening gain. $T_c$ is constant that describes how long it will take to reach the normal positions $\Delta \phi = 0$ and
$\Delta \theta = 0$ respectively if there is no error.

The modified reference frame $R^{d*}_c$ can the be calculated by rotating the reference frame by the additional angles:

\begin{equation}
R^{d*}_c = R^d \cdot R_{RPY}(\Delta \phi, \Delta \theta, 0)
\end{equation}

To get an idea how this controller compensates CoM inaccuracies consider the case where the upper body is bent forward.
Since our reference trajectory specifies an upright upper body pose we can assume that $\phi^d = 0$.
Since the upper body is bent forward the roll angle $\phi$ will be below zero.
Depending on $D_c$ we will eventually reach $\Delta \phi \approx |\phi|$, thus the reference frame will be modified to bent backwards
to compensate the wrong pose.


### Controlling the ankle torques

Since the stabilizer only has the joint trajectory and desired ZMP trajectory as input,
we need a way to compute the desired actuation torques on the ankles.
The canonical way to do this, would be to solve the inverse dynamics of the robot.
However for this we need an acurate model of the robot, including correct masses and moments of intertia for each link.
This model is not always easy to obtain and calculating the inverse dynamics of a robot with many degrees
of freedom is rather slow.
For this reason a simple heuristic is proposed to yield approximate torques given a reference ZMP position.
However in the single support phase it is easy to calculate the *exact* actuation torque on the ankle, that is required to realize the given reference ZMP.

First we need to calculate the force in $z$-direction applied on the foot at the ankle $p_{ankle}$ which we name $f_g$ by:

\begin{equation}
f_g = M \cdot g
\end{equation}

Where $g$ is the gravity vector and $M$ the mass of the robot.
Given $f_g$ acting on the ankle position $p_{ankle}$ we can obtain the ankle torque in single support phase easily using the fact,
that the torque around the ZMP is zero:

\begin{equation}
\begin{array}{lcccr}
\tau_{zmp} & = & (p_{ankle} - p^d_{zmp}) \times f_g & + & \tau^d_{ankle} \\
0 & = & (p_{ankle} - p^d_{zmp}) \times f_g & + & \tau^d_{ankle} \\
\tau^d_{ankle} & = & -(p_{ankle} - p^d_{zmp}) \times f_g & & \\
\end{array}
\end{equation}

In dual support phase however that matter is more complicated. Since both
feet are in contact with the ground, the weight of the robot is distributed between them.
If we take the forces $f_R$ and $f_L$ which act on the right ankle $p_R$ and left ankle $p_L$ respectively
we know that $f_R + f_L = f_g$. Thus there exists $\alpha \in [0, 1]$ for which:
$f_R = \alpha \cdot f_g$ and $f_L = (1-\alpha) \cdot f_g$.
A heuristic for computing this alpha is the *ZMP distributor*.

The idea is to calculate the nearest points $p_{L\#}$ and $p_{R\#}$ from the ZMP to the support polygones of the feet.
If the ZMP falls inside one of the support polygones set $\alpha = 1$ or $\alpha = 0$ respectively.
If it is outside of bothe support polygones the ZMP is projected onto line from $p_{L\#}$ to $p_{R\#}$ yielding the point $p_{\alpha}$.

We can then set $\alpha$ to:

\begin{equation}
\alpha = \frac{|p_{\alpha} - p_{L\#}|}{|p_{R\#} - p_{L\#}|}
\end{equation}

If $\tau_L$ and $\tau_R$ are the torques in the left and right ankle respectively, we can calculate the torque around the ZMP as:

\begin{equation} \label{eq:ds-torque}
\tau_{zmp} = (p_R - p^d_{zmp}) \times f_R + (p_L - p^d_{zmp}) \times f_L + \tau^d_L + \tau^d_R
\end{equation}

As before, we assume that $\tau_{zmp} = 0$ which lets us solve \ref{eq:ds-torque} for $\tau_0 := \tau^d_L + \tau^d_R$:

\begin{equation} \label{eq:tau0-torque}
\tau_0 = (p_R - p^d_{zmp}) \times f_R + (p_L - p^d_{zmp}) \times f_L
\end{equation}

We now again apply a heurstic using the $\alpha$ computed before to distribute $\tau_0$ to each ankle.
First we need to transform $\tau_0$ from the global coordinate system to a local coordinate system
described by the *ground frame*. We mark all vectors in the local coordinate system with $'$.
The heuristic applied is: The torque around the $x$-Axis in each ankle is approximately proportional to
the force applied at that ankle. Thus:

\begin{equation} \label{eq:torque-right-x}
\tau^{d'}_{Rx} = \alpha \tau_{0x}'
\end{equation}
\begin{equation} \label{eq:torque-left-x}
\tau^{d'}_{Lx} = (1-\alpha) \tau_{0x}'
\end{equation}

The torque around the $y$-Axis depends on the direction of the total torque $\tau_{0y}'$. If the total torque
acts in clockwise direction (negative sign), we can assume it will only be applied to the left foot.
If the torque acts in anti-clockwise direction (positive sign), we assumte it will only be applied to the right foot.

\begin{equation} \label{eq:torque-right-x}
\tau^{d'}_{Ry} = \left\{
\begin{array}{lr}
\tau_{0y}', & \tau_{0y'} > 0 \\
0, & else
\end{array}
\right.
\end{equation}
\begin{equation} \label{eq:torque-left-x}
\tau^{d'}_{Ly} = \left\{
\begin{array}{lr}
\tau_{0y}', & \tau_{0y'} < 0 \\
0, & else
\end{array}
\right.
\end{equation}

We can now transform the torques form our local coordinate system to the coordinate system of the corresponding foot
yielding $\tau^d_L$ and $\tau^d_R$.

Now that we have obtained the reference torques, we can try to control the torque in each angle
using the measured torques $\tau_R$ and $\tau_L$. However since we assume a position controlled robot,
the torque differences need to be translated into pose changes.
There are three primary cases that need to be considered if we change the reference pose of a foot:

\todo{image with springs}

The foot is not in contact with the ground:
  ~ Changing the reference pose will just affect the foot

The foot is in contact with the ground, but the contact is non-solid:
  ~ If the foot has a soft contact surface (e.g. rubber) we can model the contact
    with the ground as springs that connect the ground with the contact points on the foot.
    Changing the pose of the foot will relax/compress the springs and change the contact forces acoordingly.

The foot is in solid contact with the ground:
  ~ Changing the reference foot pose *will not change the foot pose at all*.
    Instead, the pose of the rest of the robot is changed. If the foot pose is changed by the angles $\Delta \phi$ and $\Delta \theta$
    all other frames of the robot will be changed by $-\Delta \phi$ and $-\Delta \theta$.

For a foot with a rubber surface we will start with a non-solid contact and transition to a solid contact, once the rubber is sufficiently compressed.
\cite{kajita2005running}

To get an idea how changing the pose on such a foot with rubber surface affects the torque,
consider the case of rotating the foot around its lateral axis ($x$-Axis) in anti-clockwise direction.
Since the contact with the ground is at first non-solid, we can employ the spring model. The springs at the front of the foot are compressed
thus the force applied at the corresponding contact points increases.
Accordingly the springs at the back are compressed less, thus the force applied to the corresponding contact points decreases.
Resulting we see a increase in torque around the $x$-Axsis.

If the springs are compressed sufficiently, we can assume the contact with the floor is solid. Since the pose of the foot does not change,
the increase in the joint angle in the ankle will rotate the upper body backwards. Recall the 3D-LIMP model for a moment,
in that model this means our pendulum swings backwards. This will lead to an increase of the torque around the $x$-Axis in the base of the pendulum, the ankle joint.

For rotating the foot around the $y$-axis the same ideas hold.
As a result we see that additional foot rotation long the $x$- and $y$-Axis have a proportional relationship with the torque
around that axis. This motivates the definition of the controller proposed by Kajita et. al. The additional rotation angles
are can be calculated by the following equations:

\begin{equation}
\Delta \dot{\phi_i} = \frac{1}{D_{ix}} (\tau^d_{ix} - \tau_{ix}) - \frac{1}{T_{ix}} \cdot \Delta \phi_i
\end{equation}
\begin{equation}
\Delta \dot{\theta_i} = \frac{1}{D_{ix}} (\tau^d_{iy} - \tau_{iy}) - \frac{1}{T_{ix}} \cdot \Delta \theta_i
\end{equation}

Where $i \in \{R, L\}$. This again utilizes the same concept of a dampening controller
that was used previously for controlling the chest frame orientation.
We can use the obtained angles $\Delta \phi_i$ and $\Delta \theta_i$ to compute the modified reference frames for the feet:

\begin{equation}
R^{d*}_i = R^d_i \cdot R_{RPY}(\Delta \phi_i, \Delta \theta_i, 0)
\end{equation}

### Foot force difference controller

In the previous section only the ankle torque were controlled to match the reference values that were derived using the ZMP distributer.
However reference values for the gravitational force that each foot exerts on the ground were also obtained.
These forces are not necessarily realized. Consider the case of slightly uneven ground. If the pattern assumed a flat ground,
the ankle of both feet will have the same altitude. Depending on variation of floor height, that might lead to one foot not touching the ground at all.
In the case of feet with rubber soles, slight variations in floor height lead to a different compression of the soles. Both cases cause a different force acting on each foot.

If we assume the mass $M$ of the robot and the gravity vector $g$ are correct, we know that the reference force $f^d_g = M \cdot g$
will exactly match the force in $z$-direction $f_g$ exerted by the support foot in single support.
Thus in single support we can guarantee that we realize our reference force.
In dual support we know that $f^d_L + f^d_R = M \cdot g$. If we apply the same reasoning as above we know that $f^d_L + f^d_R = f_L + f_R$.
If we can additionally make sure that $f^d_L - f^d_R = f_L - f_R$ we can deduce that $f^d_L = f_L$ and $f^d_R = f_R$.
Equation \ref{eq:ds-torque} can be used to calculate the ZMP position from the applied forces and torques on the foot. If both reference torques and reference forces on the feet are realized, that will guarantee the reference ZMP is also realized.

Since the $x$ and $y$ components of both $f^d_L - f^d_R$ and $f_L = f_R$ are zero we only need to control the $z$
components. As we motivated in the beginning of this section, differences in floor height are the main cause of deviation in the force.
To compensate that, the height of the ankle needs to be changed.
Thus the difference in ankle height $z_{ctl}$ was chosen to compensate the difference in forces
exerted by the foot. The description of the controller again uses the concept of a dampening controller, that was used in the previous sections.

\begin{equation}
\dot{z}_{ctl} = \frac{1}{D_z} [(f^d_L - f^d_R) - (f_L - f_R)] - \frac{1}{T_z} z_{ctl}
\end{equation}

Two methodes were proposed to realize this difference in ankle height.
The first methode is straight forward change the reference position of the feet in $z$-direction:

\begin{equation}
p^{d*}_R = p^d_R + 0.5 \cdot \left(\begin{array}{c}0 \\ 0 \\ z_{ctl} \end{array}\right)
\end{equation}
\begin{equation}
p^{d*}_L = p^d_L - 0.5 \left(\begin{array}{c}0 \\ 0 \\ z_{ctl} \end{array}\right)
\end{equation}

This can lead to singularities if both legs are already fully streched, as the edge of their workspace is reached.
The second methode relies on an additional rotating the pelvis link. For this approach to work, the robot needs
a joint that allows rotations around the anterior axis ($y$-Axis) to keep the upper body uprigt.
Since the robot model we used does not have this DOF, we only implemented the first approach.

### Interaction between controllers

While each controller operate independently, their effects are highly coupled.
The most important coupling exists between the chest posture controller and the ankle torque controller.
Recall that in case of a solid contact with the ground the ankle torque controller will not rotate the supporting foot
but rather the body of the robot. This will however change the posture of chest frame. The chest posture controller
compensates that and keeps the body upright.
This tight coupling makes tuning the parameters $D_i$ and $T_i$ of the controllers difficult, as their performence depends on the other controllers.
Best results where observed when the chest posture controller was tuned independently first, disabling the other controllers.
Then the foot force controllers was enabled and tuned and finally the ankle torque controller was added and tuned.

### CoM and ZMP control

The controllers specified in the previous sections can make sure,
that the ZMP that is realized tracks the ZMP that would result from a perfect execution of the input pattern.
However depending on how the reference ZMP was predicted, that prediction might have already been wrong.
For example the ZMP Preview Control approach uses the Table-Cart model to predict the ZMP. That prediction
can deviate significantly from the real ZMP as the model simplifies the dynamics.
Thus to make sure the desired ZMP is tracked acurately, the reference ZMP needs to be adapted as well.

Kajita et. al. propose a dynamic system that describes the 3D-LIMP dynamics. To model mechanical lag they introduce a
parameter $T_p$ that should specify the ZMP delay. The state-space description of the dynamic system for the $x$-direction is given below.
As before, the description of the dynamic system for the $y$-direction is analogous.

\begin{equation} \label{eq:dyn-system-adaption}
\frac{d}{dt} \left(\begin{array}{c}
c_x \\
\dot{c}_x \\
p_{zmp_x} \\
\end{array} \right)
=
\overbrace{
\left(\begin{array}{ccc}
0 & 1 & 0\\
\frac{g}{z_c} & 0 & -\frac{g}{z_c} \\
0 & 0 & -\frac{1}{T_p} \\
\end{array}\right)
}^{ =: A}
\cdot
\left(\begin{array}{c}
c_x \\
\dot{c}_x \\
p_{zmp_x} \\
\end{array}\right)
+
\overbrace{
\left(\begin{array}{c}
0 \\
0 \\
\frac{1}{T_p} \\
\end{array}\right)
}^{ =: B}
u
\end{equation}

As controller a feedback controller is proposed:

\begin{equation}
p^{d*}_x = u = (k_1, k_2, k_3) \cdot
\left[
\left(\begin{array}{c}
c^d_x \\
\dot{c}^d_x \\
p^d_{zmp_x} \\
\end{array}\right)
-
\left(\begin{array}{c}
c_x \\
\dot{c_x} \\
p_{zmp_x} \\
\end{array}\right)
\right]
+ p^d_{zmp_x}
\end{equation}

To derive the corresponding gains $(k_1, k_2, k_3)$ pole-placement with the poles
$(-13, -3, \sqrt{\frac{g}{c_z}})$ was proposed. The gains can be easily computed from
the poles, $A$ and $B$ using predefined functions in \name{MATLAB} or similar software.

## Implementation

To implement the stabilizer above, a number of problems had to be solved.
For one, computing the inverse kinematics is challenging. During walking the base of support
depends on which foot is in contact with the ground. For example, if the left foot is the support foot, it is considered the base
and the right foot is considered the TCP.
The reverse is true if the right foot is the support foot.
In dual support phase, we actually have two bases of support, which yields a parallel kinematic chain.
\name{Simox}, the framework used to compute the inverse kinematics, describes the kinematic structure of a robot
as a directed tree. Solving the inverse kinematics was initially only possible for sub-trees of that tree.
Which means it was not possible to chose base and TCP freely, but it was determined by the structure in which the kinematic model
was initially described.

As a first approximation, a kinematic model with the left foot as root node was used.
In the case of the right foot being the support foot, this approach leads to an increased error.
Consider solving the inverse kinematics for both legs using a differential solver in two steps:
First from the base (the left foot) to the pelvis, then from the pelvis to the right foot.
Lets assume the IK computes a perfect solution to place the pevlis link and only achieves a small pose error of $e_{\alpha} = 0.1°$ in the pitch angle of the right foot.
If the trajectory is execute, the right foot will achieve its target pose, since it is constrained by the ground contact.
However the error in the right foot pose will effect all other frames of the robot. Assuming a offset of $v = (-0.5, 0, 1)^T m$ from the right foot
to the pelvis link, we can compute the realized pelvis offset as $v' = R_y(-e_{\alpha}) \cdot v = (-0.49825, 0,  1.00087)^T m$
Which yields $1.76mm$ error in x-direction and $0.87mm$ error in y-direction.

To solve this \name{Simox} was extented by a function ```cloneInversed``` that can compute a kinematic structure with abitrary root placement from an existing description.
For the parallel kinematic chain, it proofed sufficient to approximate it as a normal kinematic chain and constrain the base and TCP
targets acordingly. However since the position of one foot will have small positioning errors, there will be some jitter introduced
into the system. Integrating a solver for parallel kinematics might decrease some of the jitter observed in foot contact forces during dual support,
as the constraint solver used for the simulation will only amplify this jitter.

As was the case with the components of the pattern generator, the core of the stabilizer
is implemented as part of \name{libBipedal}. A plugin integrates the
stabilizer into the dynamic simulation.

## Problems

Some problems became immediately clear when testing the stabilizer proposed by Kajita.
The ground reaction forces in dual support are oscillating widly. Instead of a continous force at about $0.5 \cdot f_g$
the forces on both feet oscillate between $0$ and $f_g$, the support foot changes in rapid successions.
As outlines in section \ref{section:rigid-body-simulation} this is a result of the constraint solver methode employed by \name{Bullet}.
Subsequently the measured torques on the ankles did not follow the prediction as well.
Pleae note that this is not merely sensor noise, these are the actual values used by the simulator.
Even when adding mean-filters to smoothen the measured torques it was not possible to extract a meaningful control signal.
Besides the sequential impulse solver newer versions of \name{Bullet} support a solver based on the Featherstone algorithm.
Given the scope of this thesis, integrating that solver was out of question, thus an alternative approach to stabilizing
had to be found.

## Alternative approach {#section:alternative-approach}

A a simple heuristic, we used the controllers proposed by Kajita as inspiration and replaced the force and torque
feedback with the pose error of pelvis and feet frames respectively.
The chest controller Kajita et. al. proposed for controlling the body posture where adapted to
all control frames to provide a feedback on the pose error.

This yields a controller that keeps the feet pose parallel to the ground, which is important when the swing foot touches the ground.
Controlling the pelvis and chest pose to follow the reference also keeps the robot upright.
It should be noted that it is probably not feasible to implement this stabilizer in practice. As mentioned in section \ref{section:stabilizer}
precisely estimating the pose of a robot is not easy. While the dampening controllers can be configured to smoothen a noisy sensor signal,
a high level of precision is required to ensure a correct foot posture.

Since the ZMP and CoM trajectory is not adaped, the compensation of environment disturbences is only based on a fast controller reaction
to leave the reference trajectory a little as possible and the stability margins the ZMP provides.
However as we will discuss in the evaluation section, this simple approach is already supprisingly resilient.

