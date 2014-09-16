# Introduction
motivation,  and a bit of overview of humanoid walking.  I recommend to leave it for later, start with the sections that you feel its easier to write (usually, the ones that have more content).

# Models for humanoid walking

## Humanoid walking

- Support phases
- ...

## Static walking

## Dynamic walking

## The linear inverted pendulum model

\todo{Use different name for CoM, $p$ will be rather used for the ZMP, maybe $c$?}

\todo{picture of 3D-LIPM}

A simple model for describing the dynamics of a bipedal robot during single support phase is the 3D inverted pendulum.
We reduce the body of the robot to a point-mass at the center of mass and replace the support leg
by a mass-less telescopic leg which is fixed at a point on the supporting foot.
Initially this will yield non-linear equations that will be hard to control.
Howevery by constraining the movement of the inverted pendulum to a fixed plane, we can derive a linear dynamic system.
This model called the 3D *linear* inverted pendulum model (short *3D-LIPM*).

## The inverted pendulum

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

## Linear Inverted Pendulum Model

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
\ddot{z} = k_x \cdot \ddot{x} + k_y \cdot \ddot{y} + z_c
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
\begin{equation}
\ddot{y} = \frac{g}{z_c} y - m z_c \cdot u_R
\end{equation}
\begin{equation}
\ddot{x} = \frac{g}{z_c} x + m z_c \cdot u_P
\end{equation}

\todo{include pattern generation just based on 3D-LIPM, I don't understand how they derived the controller}

## The Zero Moment Point

A very popular approach to humanoid walking are schemes based on the Zero Momement Point. One reason for that might be that
it is very simple to describe constrains for dynamic stability using this reference point:
As long as the following conditions are met we will have dynamically stable walking:

1. The support foot has flat, non-sliding ground contact.

2. The ZMP is strictly inside the support polygone of the support foot.

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

This will yieal: $\tau_x = \tau_y = 0$.

Please note that $\tau_z$ will in general not be zero, nonetheless in case of straight walking it is often assumed to be zero as well.

## The table-cart model

explain the inverted pendulum model
maybe the ZMP theory also can go here.
And include here the challenges of dynamic simulators, whatever you want to add here about simulators (although you can also talk about our particular problems in the next section, in the simulation results).

# Pattern generator
Theory
    how to plan a dynamically stable movement 
    preview controller 
Implementation
Dynamic simulation

# Controllers to stabilize a trajectory
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
