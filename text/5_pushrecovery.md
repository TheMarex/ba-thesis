# Push recovery

As we saw in the chapter about Stabilizers, not all disturbances can be compensated.
If the disturbance reaches a certain severity, the trajectory can not be executed as planed without falling.
Thus the trajectory needs to be changed radically to avoid falling.
There is little hope to recover from continuous heavy disturbances, as any attempt to recover
will be defeated. Thus we focus on short but severe disturbances, pushes.

The most prominent method to recover from pushes is the Capture Point. The idea is to find
a point, that will guarantee that the CoM comes to a rest, if the support foot is instantaneously placed there.
The push recovery implemented here uses a simplistic method based on the Capture Point.

## Capture Point

Koolen et al. \cite{koolen2012capturability} derive the Capture Point for multiple models based on the 3D-LIPM.
The simplest model is the 3D Linear Inverted Pendulum with a point contact and a massless
telescopic rod.
If we use the LIP equations \ref{eq:lip-x} and \ref{eq:lip-y} with zero input torque,
we can derive that so called *orbital energy* of the pendulum.
As we did in chapter \ref{chapter:pattern-generator}, we will derive the equations
only for one dimension. The other dimension follows analogous.

The base of the pendulum is assumed to be at the origin of the reference
frame in \ref{eq:lip-x}. Since we want to specify the location of the CoM $c = (c_x, c_y, c_z)$ freely,
we need to substitute $c_x$ with $c_x - p_x$ to yield:

\begin{equation} \label{eq:lip-x-general}
\ddot{c_x} = \frac{g}{z_c} (c_x - p_x)
\end{equation}

The orbital energy $E_x$ can be derived by subtracting the potential and kineatic energy:

\begin{equation} \label{eq:orbital-x}
E_x = \overbrace{\frac{1}{2} \dot{c_x}^2}^{\text{kinetic energy}} - \overbrace{\frac{g}{2 \cdot z_c} (c_x - p_x)^2}^{\text{potential energy}}
\end{equation}

For the CoM to come to a rest the orbital energy $E_x$ must be zero.
Thus we can solve \ref{eq:orbital-x} to yield the $x$ coordinate of point $p$ that will achieve this.
Since it is a quadratic equation there are two solutions:

\begin{equation}
p_x = x - \sqrt{\frac{z_c}{g}} \dot{c_x} \:\:\:\:\text{ or }\:\:\:\: p_x = x + \sqrt{\frac{z_c}{g}} \dot{c_x}
\end{equation}

Since it is generally desirable to chose a point that lies in the direction of the CoM motion
we define the *Immediate Capture Point* as:

\begin{equation} \label{eq:icp}
p_{ic} := c_x + \sqrt{\frac{z_c}{g}} \dot{c_x}
\end{equation}

Placing the base of the pendulum (the ankle) there *instantaneously* will cause
an orbital energy of zero, thus the head of the pendulum (the CoM) will stop at $p_{ic}$.
In most cases we will not be able to move the base of the pendulum instantaneously.
So we are more interested in the Immediate Capture Point in $\Delta t$ seconds from now.
This point can be obtained by \ref{eq:future-icp}.
For a detailed derivation, we recommend the paper by Koolen et al. \cite{koolen2012capturability}.

\begin{equation} \label{eq:future-icp}
p_{ic}(\Delta t) = p_{ankle} + (p_{ic}(0) - p_{ankle}) \cdot e^{\frac{g}{z_c} \cdot \Delta t}
\end{equation}

Where $p_{ankle}$ is the current position of ankle of the support foot (the current base of the pendulum).
and $p_{ic}(0)$ is the current Immediate Capture Point as calculated by equation \ref{eq:icp}.

## Fall detection

The first important step to recover from a large disturbance is to detect it.
That means we need to detect if we reached a unstable state, from which it is unlikely that we can
recover only by means of the stabilizer.
We can use the measured ZMP to obtain a heuristic for such states. If the ZMP is
outside or on the edge of the support polygon, the Cart-Table model does not guarantee stability.
However in normal operations it is likely that the ZMP will touch (or leave) the border of the support polygon
for short periods of time. A simple method to filter out this noise is to define a minimum duration the ZMP
as to be in an unstable state.
Choosing a small value will let us react faster to disturbances, but make the method error prone.
Choosing a large value will add an additional delay until we can react, but is much less prone to be triggered by
mistake. An experimental evaluation yielded good results with a duration of $t = 30ms$.

## Recovery

If a fall is detected, a recovery maneuver needs to be executed.
In single support phase, that means we need to place the swing foot at the capture point.
In dual support phase we try to move the support foot that is closest to the capture point.
Recall that the base of the pendulum coincides with the ZMP in our model view.
Thus if the resulting support polygon includes the capture point, the position will be stable.
Since the motor velocities are limited, we need a minimal amount of time $t_{min}$ to place
the foot in the desired location.
Thus instead of using the Immediate Capture Point, we use the future Immediate Capture Point $p_{ic}(t_{min})$
to derive the desired location. A value of $t_{min} = 0.35s$ yielded good results.

## Implementation

The stabilizer that is outlined in chapter \ref{section:stabilizer} was
extended to call the fall detection module in each iteration of the control loop.
If an unstable state is reached, the recovery module overrides the normal trajectory
of the stabilizer. After completing the recovery maneuver, the robot remains
in the given position.
Resuming the original pattern requires planning a dynamically stable transition trajectory.
This is left for future work.

