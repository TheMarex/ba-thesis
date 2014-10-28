# Conclusion and future work

the design and implementation of a software framework
for bipedal humanoid walking were presented. Building on an implementation by Ömer Telemz,
a generator for walking pattern using ZMP Preview Control was developed.
To test the resulting patterns a dynamic simulation for walking was build.
The simulation utilizes the \name{SimDynamics} framework contained in \name{Simox}.
The simulation was extended by real-time controllers for stabilization and push recovery.
To facilitate easy integration into other software projects all methods presented here
are implemented in a shared library \name{libBipedal}. \name{libBipedal} is independent
of the dynamic simulation and only depends on \name{VirtualRobot} for computing
forward and inverse kinematics.

Evaluation showed that stable walking could be realized in simulation.
By applying a stabilizer the trajectories remained stable, even when faced
with disturbances.
Falling was avoided by initiating a push recovery mechanism based on the Capture Point.

Since accuracy of the dynamic simulation suffered severely by using the Sequential Impulse Solver method,
it should be replaced by a solver based on the Featherstone method.
The improved accuracy should make a torque feedback more viable and enable
to actually use the stabilizer based described and implemented in \ref{section:stabilizer}.

The pattern generator and stabilizer only realize basic walking that is notably different to human walking.
For one the knees remain bent at all times, the center of mass stays at the same height and the toe joint is not
used. A follow up paper \cite{kajita2012evaluation} extends the methods implemented here to include the toes.
The realized walking trajectories seem to be more natural.

Englsberger et al. \cite{englsberger2011bipedal} propose a pattern generator based on the Immediate Capture Point instead
of the ZMP. Comparing the performance of both approaches could be worthwhile.

As the evaluation of the circular trajectory shows, the yaw moment exerted on the foot can cause severe disturbances.
To better deal with trajectories that include turns or arm movements, the resulting torque should be compensated. Kim et al.\cite{kim2005humanoid} propose to move the arms around the roll axis to compensate for yaw momentum.

Another interesting approach is to use the angular momentum around the center of mass as proposed by Kajita et al. \cite{kajita2001balancing}
and extended by Komura et al. \cite{komura2005feedback} as control input.

The push recovery implemented is very rudimentary. The placement of the foot to recover from a push does not consider collisions
with the environment. Also, after executing a push recovery step, the original trajectory can not be resumed.
To enable that, online planning of dynamically stable motions needs to be integrated.
Also the foot is placed at the exact location of the future capture point. However it suffices to place
the foot in a way to include the capture point. In most cases this would requires a lot less foot movement.
Moving the leg to the target point also changes the velocity of the CoM, thus it changes the location of the capture point.
To compensate for this, the capture point should be tracked using a controller instead of using a static future capture point.

<!--
# TODO

\listoftodos

-->
