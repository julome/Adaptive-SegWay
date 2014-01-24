Adaptive-Control-SegWay
========================

Adaptive predictive control with an Atmega168

The objective is the implementation of an adpative predictive control at a low cost as Arduino microcontroller.

This is an example of an adaptive predictive control implemented in a balancing robot using a MPU6050 and an Atmega168 (Arduino Por Mini).


Firstly, i am using a complementary filter, with a period control of 14ms, for read the balancer angle, secondly i am using an adaptive predictive control with a prediction horizon of 5 control periods, and the model predictive has a delay of one control period.

In this robot is controled roll and yaw position, therefore, it is always in balancing mode and is always in the same direction.

We can see how even the predictive adaptive control adapts to noise getting a satisfactory control of the process. In this video you can see the result: http://youtu.be/K3qdtP572tM

Visit my project's blog in: http://cuadricopteroadaptativo.blogspot.com
