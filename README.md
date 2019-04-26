# IMU-algorithm
A sensor fusion algorithm to determine roll and pitch in 6-DOF IMU.

# Demo
Demonstration using an Arduino Uno, MPU6050 (without DMP support) and Processing software. 

![Alt text](https://github.com/rbv188/IMU-algorithm/blob/master/demo_gifs/gif_1.gif)

# Working
The orientation is calculated as a quaternion that rotates the gravity vector from earth frame to sensor frame. The gravity vector in the sensor frame is the accelerometer readings and the gravity vector in earth frame is (0,0,-1).

The accelerometer values are sensitive to vibrations. The gyroscope is used to keep track of the gravity vector and correct the accelerometer readings.

# Usage
To use the library, read the accelerometer, gyro values, and calculate the time taken to complete a loop.

```C
/*
fused_vector - corrected accelerometer readings
delta - delay or time taken to complete a loop
wx,wy,wz - gyro values in rad/s
AcX, AcY, AcZ - raw accelerometer values
q_acc - quaternion representing orientation
angles - euler angles
*/

delta = 0.001*(millis()-Start);
fused_vector = update_fused_vector(fused_vector,AcX,AcY,AcZ,wx,wy,wz,delta);
  
q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
angles = quaternion_to_euler_angles(q_acc);
```

The library uses an approximation for 1/sqrt(x) (in vector_3d.c) which may not work on all machines. The definition of the function may have to be changed in some hardware.  

# Reference/Credits/Sources

[Beautiful maths simplification: quaternion from two vectors](http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors)

[Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs](https://www.mdpi.com/1424-8220/15/8/19302)

[Phillip's Technology Corner - Fast Quaternion Integration for Attitude Estimation](https://philstech.blogspot.com/2014/09/fast-quaternion-integration-for.html)

[Pizer’s Weblog - Fast Inverse Square Root](https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/)

[Processing Code](https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser)
