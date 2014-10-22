# Models for humanoid walking {#section:walking-models}

## The Linear Inverted Pendulum Model

A simple model for describing the dynamics of a bipedal robot during single support phase is the 3D inverted pendulum.
We reduce the body of the robot to a point-mass at the center of mass and replace the support leg
by a mass-less telescopic leg which is fixed at a point on the supporting foot.
Initially this will yield non-linear equations that will be hard to control.
However by constraining the movement of the inverted pendulum to a fixed plane, we can derive a linear dynamic system.
This model called the 3D *linear* inverted pendulum model (short *3D-LIPM*).

\begin{wrapfigure}{R}{0.4\textwidth}
  \begin{center}
     \includegraphics[width=0.4\textwidth]{images/3dlimp.png}
  \end{center}
  \caption{The 3D-LIMP}
\end{wrapfigure}

### The inverted pendulum

To describe the dynamics of the inverted pendulum we are mainly interested in the effect a given actuator torque has on the movement of the pendulum.

For simplicity we assume that the base of the pendulum is fixed at the origin of the current Cartesian coordinate system.
Thus we can describe the position inverted pendulum by a vector $c = (x, y, z)$.
We are going to introduce an appropriate (generalized) coordinate system $q = (\theta_x, \theta_y, r)$ to get an easy description of our actuator torques:
Let $m$ be the mass of the pendulum and $r$ the length of the telescopic leg.
$\theta_y$ and $\theta_x$ describe the corresponding roll and pitch angles of the pose of the pendulum.

Now we need to find a mapping between forces in the Cartesian coordinate system and the generalized forces (the actuator torques).
Let $\Phi: \mathbb{R}^3 \longrightarrow \mathbb{R}^3, (\theta_x, \theta_y, r) \mapsto (x, y, z)$ be a function that maps the generalized coordinates to the Cartesian coordinates.
Then the Jacobian $J_\Phi = \frac{\partial p}{\partial q}$ maps the *generalized velocities* to *Cartesian velocites*.
Furthermore we know that the transpose $J_\Phi^T$ maps *Cartesian forces* $F = m (\ddot x, \ddot y, \ddot z)$ to *generalized forces* $\Gamma = (\tau_x, \tau_y, f)$.

We write $x$, $y$ and $z$ in terms of our generalized coordinates to compute the corresponding Jacobian $J_\Phi$.
From the fact that the $\theta_y$ is the angle between the projection of $c$ onto the $xz$-plane and $c$
and $\theta_x$ the angle between $c$ and the projection onto the $yz$ plane we can derive the following equations \cite{kajita20013d}.
We use $s_i = sin(\Theta_i)$ and $c_i = cos(\Theta_i)$ with $i \in \{x, y\}$ as a shorthand notation.

\begin{equation}
\begin{array}{lcll} \label{eq:lip-xyz}
x & = & r \cdot \sin \theta_y & =: r \cdot s_y\\
y & = & -r \cdot \sin \theta_x & =: -r \cdot s_x \\
z & = & \sqrt{r^2 - x^2 - y^2} = r \cdot \sqrt{1 - s_y^2 - s_x^2} & \\
\end{array}
\end{equation}

From which we can compute the Jacobian by partial derivation:

\begin{equation} \label{eq:lip-Jacobian}
J = \frac{\partial p}{\partial q} = \left( \begin{array}{rcl}
0 & r \cdot c_y & s_y \\
-r \cdot c_x & 0 & s_y \\
\frac{2 \cdot r \cdot s_y c_y}{\sqrt{1 - s_y^2 - s_x^2}} & \frac{2 \cdot r \cdot s_x c_x}{\sqrt{1 - s_y^2 - s_x^2}} & \sqrt{1 - s_y^2 - s_x^2}\\
\end{array}
\right)
\end{equation}

Using the gravity force $f_g = (0, 0, -m \cdot g)^T$ and the equation of motion as given by
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
\tau_x \\
\tau_y \\
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

and equations \ref{eq:lip-Jacobian} and \ref{eq:lip-xyz} we can derive the following equations:

\begin{equation} \label{eq:lip-dyn-y}
m(-z\ddot{y} + y\ddot{z}) = \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_x} \cdot \tau_x + m g y
\end{equation}

\begin{equation} \label{eq:lip-dyn-x}
m(z\ddot{x} - x\ddot{z}) = \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_y} \cdot \tau_y + m g x
\end{equation}

Observe that the terms of the left-hand side are not linear. To remove that non-linearity
we are going to use the *linear* inverted pendulum model.

### Linearization

In a man-made environment it is fair to assume that the ground a robot will walk on
can be approximate by a slightly sloped plane. In most cases it can even assumed that there is no slope at all.

The basic assumption in the next section will be that the CoM will have a *constant displacement* with regard to our ground plane.
Thus we can constrain the movement of the CoM to a plane that is parallel to the ground plane.
Note that this assumption is, depending on the walking speed, only approximately true for human walking as shown by Orendurff et al. \citep{orendurff2004effect}.
For slow to fast walking ($0.7$ m/s and $1.6$ m/s respectively) the average displacement in $z$-direction was found to be between $2.7cm$ and $4.81$ cm.
While the walking patterns generated based on the LIP-model will guarantee dynamic stability, they might not look natural with regard to human walking.

We are going to constrain the $z$ coordinate of our inverted pendulum to a plane
with normal vector $(k_x, k_y, -1)$ and $z$-displacement $z_c$:

\begin{equation} \label{eq:lip-z-plane}
z = k_x \cdot x + k_y \cdot y + z_c
\end{equation}

Subsequently the second derivative of $z$ can be described by:

\begin{equation} \label{eq:lip-z-div}
\ddot{z} = k_x \cdot \ddot{x} + k_y \cdot \ddot{y}
\end{equation}

Substituting \ref{eq:lip-z-plane} and \ref{eq:lip-z-div} into the equations \ref{eq:lip-dyn-y} and \ref{eq:lip-dyn-x}
yields the following equations:

\begin{equation}
\ddot{x} = \frac{g}{z_c} x + \frac{k_y}{z_c} (x\ddot{y} - \ddot{x}y) + m z_c \cdot \tau_y \cdot \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_y}
\end{equation}
\begin{equation}
\ddot{y} = \frac{g}{z_c} y - \frac{k_x}{z_c} (x\ddot{y} - \ddot{x}y) - m z_c \cdot \tau_x \cdot \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_x}
\end{equation}

The term $x\ddot{y} - \ddot{x}y$ that is part of both equations is still causing the equations to be non-linear.
To make this equations linear we will assume that our ground plane has no slope, thus $k_x = k_y = 0$ and the non-linear terms will vanish.

Another problem is that the actuator torques $\tau_x$ and $\tau_y$ both have non-linear factors $\frac{\sqrt{1 - s_y^2 - s_x^2}}{c_x}$ and $\frac{\sqrt{1 - s_y^2 - s_x^2}}{c_y}$ respectively. This can be solved by substituting with the following *virtual inputs*:

\begin{equation}
\tau_x \cdot \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_x} = u_x
\end{equation}
\begin{equation}
\tau_y \cdot \frac{\sqrt{1 - s_y^2 - s_x^2}}{c_y} = u_y
\end{equation}

Which yields our final description of the dynamics:
\begin{equation} \label{eq:lip-x}
\ddot{x} = \frac{g}{z_c} x + \frac{u_x}{m z_c}
\end{equation}
\begin{equation} \label{eq:lip-y}
\ddot{y} = \frac{g}{z_c} y - \frac{u_y}{m z_c}
\end{equation}

As outlined in \cite{kajita20013d} the inputs $u_y$ and $u_x$ are generally set to zero.
Thus the 3D-LIMP has no input torque. This is desirable, as the torque
that can be applied on the ankle joints is limited. Thus it makes sense to reserve the torque
for correcting disturbances.

## The Zero Moment Point

The Zero Moment Point is virtual point on the floor that can be used to derive dynamically stable walking.
If the support foot (or support feet) have flat ground contact, the walking trajectory is dynamically stable if the ZMP
is strictly inside the support polygon. If the ZMP is on the border of the support polygon, the trajectory can be dynamically unstable.
By deriving a control scheme that constrains the ZMP to be strictly inside the support polygon, we can use the ZMP to generate dynamically
stable trajectories.

For flat ground contact of our support foot with the floor the ZMP corresponds with the position of the center of pressure (CoP).
Indeed, some author (notably Pratt) prefer to use the term CoP instead of ZMP. In the context of this thesis,
we will use the term ZMP.

The CoP (and in flat ground contact the ZMP) of an object in contact with the ground can be computed as the sum of all contact points $p_1, \dots, p_n$ weighted by the forces in $z$-direction $f_{1z}, \dots, f_{nz}$ that is applied:

\begin{equation} \label{eq:zmp-definition}
p := \left(\begin{array}{c}
p_x \\ p_y \\ p_z
\end{array}\right)
   = \frac{\sum^N_{i=1}p_i f_{iz}}{\sum^N_{i=1} f_{iz}}
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
and substitute in the equations \ref{eq:zmp-torque-x} and \ref{eq:zmp-torque-y}.

This will yield: $\tau_x = \tau_y = 0$.

Please note that $\tau_z$ will in general not be zero, nonetheless in case of straight walking it is often assumed to be zero as well.

## The cart-table model {#section:cart-table}

\begin{wrapfigure}{R}{0.4\textwidth}
  \begin{center}
     \includegraphics[width=0.4\textwidth]{images/carttable.png}
  \end{center}
  \caption{The Cart-Table model.}
\end{wrapfigure}

The cart-table model is used to compute the resulting ZMP from an CoM motion.
The model consists of an infinitely large mass-less table of height $z_c$, while the foot of the table has the shape of the support polygon.
Given a frictionless cart with mass $m$ at position $c = (c_x, c_y, c_z)$ that moves on the table we can compute the resulting ZMP $p = (p_x, p_y, p_z)^T$ in the support foot.
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

Besides the simplified cart-table model, there also exists an exact method
to calculate the resulting ZMP from the movement from several connected rigid bodies.

Let $c_i$ be the CoM position and $m_i$ the mass of the $i$-th body ($i \in \{1, ..., k\}$). Then, as derived in \cite{siciliano2008springer} (p. 374),
the total linear momentum $\mathcal{P}$ can be calculated by:

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

See figure \ref{zmp-comparison} to get an idea how much the Multi-Body ZMP
derives from the estimation using the Cart-Table model.

\begin{figure*}[tb]
\includegraphics[width=\textwidth,resolution=300]{images/zmp_comparison.png}
\caption{Comparison of the Cart-Table and Multi-Body to estimate the realized ZMP during walking simulation.}
\label{img:zmp-comparison}
\end{figure*}

