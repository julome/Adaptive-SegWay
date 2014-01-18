Adaptive-Control-SegWay
========================

Adaptive predictive control with Arduino

The objective is the implementation of an adpative predictive control at a low cost as Arduino microcontroller.

This is an example of an adaptive predictive control implemented in a balancing robot using a MPU6050 and an Atmega168 (Arduino Por Mini).

In this project I used a prediction horizon of 5 control periods, with a delay of one control period, and a control period of 14ms. The gyroscope and accelerometer readings are performed every 2ms filtering only the accelerometer, for later each 6ms accelerometer measures through a complementary type Kalman filter. Finally gets a filtered measure each 12ms moment in which the predictive adaptive function is called. All filters in the MPU6050 have been disabled by enabling a high speed of response. 

We can see how even the predictive adaptive control adapts to noise getting a satisfactory control of the process. In this video you can see the result: http://youtu.be/K3qdtP572tM

Visit my project's blog in: http://cuadricopteroadaptativo.blogspot.com
