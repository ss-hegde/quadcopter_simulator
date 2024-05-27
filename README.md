# quadcopter_simulator

This project aims to build a simulator for simulating quadrotor drone behavior during aggressive maneuvers and motor failure scenarios. This is an ongoing work, and the project will be updated regularly.

The 6-DOF model of the quadrotor drone is developed according to the methods described in [1] and [2]. The controller design follows the approach detailed in [1].

The inputs are provided using a control inceptor, and the output is visualized in real time. The output plots the commanded values in comparison with the state values. The inner loop controller manages the attitude, while the outer loop controller tracks the position.

Currently, there is a phase lag between the input and output, and the model tends to become unstable under certain conditions. These issues will be addressed and fixed in the next commit.

# References
[1] B. Prabhakaran, M. Kothari, and Abhishek, "Nonlinear Control Design for Quadrotors," Department of Aerospace Engineering, Indian Institute of Technology Kanpur. Available: https://ieeexplore.ieee.org/document/7495504.
[2] P. Wang, Z. Man, Z. Cao, J. Zheng, and Y. Zhao, "Dynamics Modelling and Linear Control of Quadcopter," in Proc. 2016 Int. Conf. Advanced Mechatronic Systems, Melbourne, Australia, Nov. 30 - Dec. 3, 2016. Available: https://ieeexplore.ieee.org/document/7813499.
