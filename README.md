# Robot Control using PID Controller and Kalman Filter
The objective is to estimate the Velocity using a Linear Kalman Filter and to control a robot using a PID Controller.

Consider a robot on frictionless, straight rails. Initially, the robot is stationary at position 0. We measure the position of the robot every Δt seconds, but these measurements are imprecise; we want to maintain a model of the robot's position and velocity. 

We show here how we derive the model from which we create our Kalman filter.
Since F,H,R and Q are constant, their time indices are dropped.

The position and velocity of the truck are described by the linear state space
x_k = [x,v]     % x : position
                % v : velocity
                
We assume that between the (k − 1) and (k) timestep, uncontrolled forces cause a constant acceleration of a(k) that is normally distributed with mean 0 and standard deviation σa. From Newton's laws of motion we conclude that
x(k) = F x(k-1) + G a(k)

We suppose there is no G a(k) term since there are no known control inputs, where
F = [1 Δt;0 1]
G = [Δt^2/2;Δt]

Then, a PID controller is used to generates the acceleration (a) to control the robot's velocity (v) using the estimated one ( $\tilde{v}$ ). 

<p align="center">
  <img src="https://github.com/SaysWis/Robot_Control/blob/main/Control_scheme.png">
</p>
