# Conclusions

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

The pattern generator and stabilizer only realize basic walking, that is not too similar to human walking.
For one the knees remain bent at all times, the center of mass stays at the same height and the toe joint is not
used. Newer paper \todo{reference paper} extend the methodes implemented here to include the foot toe.
The realized walking trajectories are a lot more human-like.

The push recovery is very rudimentary. Most notably after executing a push recovery step, the original
trajectory can not be resumed. To enable that, online motion planning needs to be integrated.

As walking in a circle shows, the current implementation does not compensate the generated yaw momentum.
A yaw compensation based on \todo{reference paper} needs to be implemented.

# TODO

\listoftodos
