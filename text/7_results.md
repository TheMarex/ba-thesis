# Results

\todo{Write small introduction}

## Undisturbed walking

### Walking in a straight line

The first scenario is walking in a straight line for 10 steps.
Figure \ref{img:player-undisturbed-straight-thumbs} shows the simulation of walking straight for 10 steps using no stabilization.

\begin{figure*}[htb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/player_undisturbed_straight_thumbs.png}
\caption{Frames of undisturbed straight walking}
\label{img:player-undisturbed-straight-thumbs}
\end{figure*}

In the unstabilized case, only the joint trajectory that is generated is interpolated
and replayed. In the stabilized case the the alternative stabiliter described in section
\ref{section:alternative-approach} is used.

Figure \ref{img:undisturbed-straight-x} shows the desired and realized CoM and ZMP trajectories
in both cases. As you can see the ZMP deviates significantly from the desired trajectory
and oscillates between both edges of the support polygone. We believe this is caused by the problems
outlined in section \ref{section:rigid-body-simulation}. Consider figure \ref{img:noisy-com-acc} which shows
the desired CoM acceleration in comparision to the realized acceleration.
However the foot remains in full ground contact and friction forces are applied acurately.
Thus dynamically stable walking is realized for both unstabilized and stabilized walking.
We hope that a more stable Featherstone constraint solver will produce more acurate contact forces and
thus ZMP values.

\begin{figure*}[hbt]
\vspace*{-1em}
\includegraphics[width=\textwidth,resolution=300]{images/undisturbed_straight_x.png}
\caption{CoM and ZMP as specified by the pattern (top) and actual realized values (middle and bottom).
All coordinates in the global reference frame.}
\label{img:undisturbed-straight-x}
\end{figure*}

\begin{figure*}[hbt]
\vspace*{-1em}
\includegraphics[width=\textwidth,resolution=300]{images/noisy_com_acc.png}
\caption{Acceleration of the CoM in $x$-direction.}
\label{img:noisy-com-acc}
\end{figure*}

### Walking in a circle

Figure \ref{img:player-undisturbed-circle-thumbs} shows the simulation of unstabilized walking in a circle.
The robot walks in 12 steps in an arc of 180° and 0.5 m radius.
As you can see the desired 180° turn is not realized completely. Due to the
chest rotation following the tangent of the circle, a torque around the yaw-axis is excerted on the foot.
Recall that during pattern generation we assumed that this torque is zero. Since we do not correct this
disturbance the trajectory deviates significantly.
However stable walking is still realized. See figure \ref{img:undisturbed-circle} for the realized ZMP
distribution and CoM trajectory for both unstabilized and stabilized walking.

\todo{image-series walking in a circle}

\begin{figure*}[hbt]
\vspace*{-1em}
% \includegraphics[width=\textwidth,resolution=300]{images/player_undisturbed_circle_thumbs.png}
\caption{\name{Armar4} walking in a half circle in 12 steps.}
\label{img:player-undisturbed-circle-thumbs}
\end{figure*}

\begin{figure}[hbt]
\vspace*{-1em}
\includegraphics[width=\textwidth,resolution=300]{images/undisturbed_circle.png}
\caption{Walking in a cricle. ZMP (left) and CoM (right) as specified by the pattern and the actually realized values.
Each for the unstabilized and stabilized case.}
\label{img:undisturbed-circle}
\end{figure}

## Disturbed walking

To test the performance of the stabilizer under disturbance, we used two
scenarios: A short push applied to the chest and slightly sloped ground.
The results are compared with the performance of the unstabilized trajectory.

### Push to the chest

To simulate a push, a ball with a radius of 11cm and weight of 450g (FIFA football) is shoot from
2m distance at the chest. \todo{force is meaningless as we don't know the units. Maybe print ball velocity on impact}
The point of impact is denoted as red lines.

\todo{actual-zmp-stabilized actual-zmp-unstabilized}
\todo{image-series}

