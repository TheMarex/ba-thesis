# Introduction

Nearly one hundred years of Science-Fiction have established the firm image
of a mechanical humanoid servant with super human capabilities, with the term
*Robot*.
Today we live in a world where large parts of our production circles are already
dominated by robots. Yet, widespread adoption of humanoid robots is still nowhere in evidence.
One core problem of humanoid robots is their inherent complexity.
Vision, cognition, manipulation and locomotion all need to be combined in one mechanism.
While each of these problems is under active research, one has been especially resilient: Locomotion.
There are multiple reasons for this. For one, locomotion is inherently complex,
as a large number of joints need to be controlled precisely.
The dynamics for such complex systems are hard to control, thus severe simplifications
need to be made to control them. And last but not least, the strategies to solve
bipedal walking are very much tied to specific problem and not easily transferable to robots
with other leg configurations.
Some robots, e.g. the \name{Armar III} robots, solve it by replacing the legs
with a stable base on wheels. While this yields convenient research platform,
the navigation in human environments is not as natural as for full humanoid robots.

Over the last two decades a lot of progress has been made in humanoid walking.
Every one is familiar with the famous \name{ASIMO} robot developed by Honda or the \name{HRP}
robots developed by \name{AIST} which demonstrate stable walking.
More recently \name{ATLAS} by Boston Dynamics shows great stability even under disturbances.
Sadly some of these platforms are closed and it is not known exactly which models
are used in each robot to derive stable walking.

As the goal for bipedal walking is to be as human-like as possible, research on
human walking is a great inspiration for robotic walking.
While the human gait can be divided into many phases, of primary interest for the stability is the number of
feet that are in contact with the ground. For walking the ground contact alternates between both feet
and one foot. These phases are called dual and single support respectively.
Each step starts with a dual support phase, that shifts the center of mass to the foot that supports the weight in the next step.
The non-supporting foot (the *swing foot*) is then moved to the next foot hold. (See figure \ref{img:human-gait})
The area that is in contact with the ground is not constant in each phase.
Most notably at the end of the single support phase the heel is lifted. Consequently the contact changes
from full sole contact to the toes.
The swing foot hits the ground heel first and rotates to meet the ground while the center of mass
is shifted forward.
\begin{figure*}[tb]
\vspace*{-1em}
\includegraphics[width=\textwidth]{images/human_gait.png}
\caption{Phases of human gait. Source: Dynamics of Human Gait \cite{vaughan1992dynamics}, page 9}
\label{img:human-gait}
\end{figure*}

Early robots where only able to emulate this walking style to varying degrees.
A common simplification is to use a foot without movable toes.
The result is a characteristic walking style, where the foot sole is always parallel to the ground.
The most prominent example is the early \name{ASIMO} robot.
However today human-like walking using the toes has been successfully demonstrated e.g. by the \name{WABIAN-2R} robot \cite{ogura2006human}.

Besides the need for an adequate kinematic structure, deriving trajectories that are stable is
not trivial.
When talking about stability, one needs to consider two cases: *static* and *dynamic* stability.
Static stability is concerned with the stability of objects at rest (e.g. standing), while dynamical stability is concerned
with the stability of objects in motion (e.g. walking).
Deriving a stability criteria for the *static* case is rather simple.
If the projection of center of mass to the ground is inside the support polygon,
the pose is said to be *statically* stable.
The support polygon is the convex hull of all points of the foot that are in contact with the ground.
However dynamic stability deals with the stability of objects (e.g. in this case robots) in motion.
In that case more elaborate methods need to be derived using a description of the dynamics of the robot.

There are two basic approaches to derive a trajectory for bipedal walking.
On the one side are approaches based on direct imitation of a human motion.
First a human test subject is equipped with markers.
The marker movements are recorded using motion capturing and mapped to corresponding
markers on a humanoid model.
Since the kinematic structure of a human does not map directly to the kinematic structure of a robot,
the resulting trajectory needs to be adapted to the target robot.
In general only adapting the motion based on the kinematic structure will not yield a dynamically stable motion.
Different weights and inertia values of the links also need to be considered.
However this might cause significant modifications of the original trajectory.

On the other side there are completely synthetic approaches. Based on simplified
models of the dynamics of a humanoid robot, stability conditions are derived.
Using desired foot positions or step length and walking speed as input, corresponding
trajectories are derived that satisfy the given stability conditions.
Most notably in this category are approaches based on the Zero Moment Point using a so called Pattern Generator.
In section \ref{section:walking-models} the ZMP is derived for different simplified
dynamic models. Fundamental for this approach is the insight, that the dynamics
of bipedal walking can be approximated sufficiently by an inverted pendulum.
The model views the center of mass as the head of the pendulum, while the base
is attached to the support foot.
Thus the task of walking can be formalized as moving the base of the pendulum
while keeping it upright.

In reality executing a trajectory that was obtained by either method is not easy.
For one, the models to approximate the dynamics of a complex robot might be inaccurate.
Also the environment the robot operates in might not match the assumptions completely.
For example a convenient assumption is that the ground is completely flat. This is rarely the
case and needs to be controlled for.
Thus a stabilizer is needed to adapt the trajectory to the disturbances.
Most stabilizer for established robotic platforms are closed source and very
robot specific. Notably various iterations of stabilizers based on modifying
an already dynamically stable pattern where proposed by Kajita and Miura et al.
(\cite{kajita2010biped}, \cite{miura2011human} and \cite{kajita2012evaluation})

While a stabilizer can deal with continuous small disturbances, a robot may
encounter short but severe disturbances. These disturbances can be viewed
as pushes. Pratt et al. established the concept of a Capture Point,
that can be used to recover from such disturbances.

This thesis presents a software framework that implements rudimentary support
for each component. They are integrated into a dynamic simulation environment
and evaluated.

