# README for overshoot behavior

# Controller type: Feedforward with PI Control

# Controller Gains: 
KP: 2
KI: 4
-This will cause an underdamped response

# Notes:
The robot initial configuration is offset from the first reference configuration. 
The initial configuration is:
phi = 0.5236
x = 0.1
y = 0.1
joint_1 = -pi/10
joint_2 = -pi/20
joint_3 = -pi/20
joint_4 = -pi/2
joint_5 = 0.0
This is to test the corrective behavior of the controller

Toward the end of the plot we see a few minor errors from Euler integration, the controller adjusts accordingly.