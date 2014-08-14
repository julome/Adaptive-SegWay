Adaptive-Control-SegWay
========================

Adaptive predictive control with an Atmega168

The objective is the implementation of an adpative predictive control at a low cost as Arduino microcontroller.

This is an example of an adaptive predictive control implemented in a balancing robot using a MPU6050 and an Atmega168 (Arduino Por Mini).


Firstly, I am using a DCM filter for calculate attitude, with a period control of 8ms, secondly I am using an adaptive predictive control with a prediction horizon of 5 control periods with a period control of 24ms, and the model predictive has a delay of two control period with 5 prediction horizon.

In this robot is controled roll and yaw position, therefore, it is always in balancing mode and is always in the same direction.

We can see how even the predictive adaptive control adapts to noise getting a satisfactory control of the process. In this video you can see the result: https://www.youtube.com/watch?v=K3qdtP572tM


