# Conclusion and future work

A design and the implementation of a software framework
for bipedal humanoid walking were presnted. Building on an implementation by Ömer Telemz,
a generator for walking pattern using ZMP Preview Control was developed.
To test the resulting patterns a dynamic simulation for walking was build.
The simulation utilizes the \name{SimDynamics} framework contained in \name{Simox}.
The simulation was extented by real-time controllers for stabilization and push recovery.
For easiy integration into other software projects all methodes presented here
are implemented in a shared library \name{libBipedal}. \name{libBipedal} is independent
of the dynamic simulation and only depends on \name{VirtualRobot} for computing
forward and inverse kinematics.

Evaluation showed that stable walking could be realized in simulation.
By applying a stabilizer the trajectories remained stable, even when faced
with disturbances.
Falling was avoided by initiating a push recovery mechanism based on the Capture Point.

Since accuracy of the dynamic simulation suffered severely by using the Sequential Impulse Solver methode,
it should be replaced by a solver based on the Featherstone methode.
The imporved accuracy should make a torque feedback more viable and enable
to actually use the stabilizer based described and implemented in \ref{section:stabilizer}.

The pattern generator and stabilizer only realize basic walking, that is noticable different to human walking.
For one the knees remain bent at all times, the center of mass stays at the same height and the toe joint is not
used. A follow up paper \cite{kajita2012evaluation} extends the methodes implemented here to include the toes.
The realized walking trajectories seem to be more natural.

As the evalutation of the circular trajectory shows, the yaw moment excerted on the foot can cause severe disturbences.
To deal better with trajectories that include turns or arm movements, that torque should be compensated. Kim et. al \cite{kim2005humanoid} propose to move the arms around the roll axis to compensate for yaw momentum.

The push recovery implemented is very rudimentary. The placement of the foot to recover from a push does not consider collisions
with the environment. Also, after executing a push recovery step, the original trajectory can not be resumed.
To enable that, online planning of dynamically stable motions needs to be integrated.

# TODO

\listoftodos
