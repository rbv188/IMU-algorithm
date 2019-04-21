#include "sensor_processing_lib.h"

Quaternion quaternion_from_accelerometer(float ax, float ay, float az)
{
    vector_ijk gravity = vector_3d_initialize(0.0f, 0.0f, -1.0f);
    vector_ijk accelerometer = vector_3d_initialize(ax, ay, az);
    Quaternion orientation = quaternion_between_vectors(gravity,accelerometer);
    return orientation;
}

Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time)
{
    // wx,wy,wz in degrees per second: time in seconds
    float alpha = 0.5*time;
    float a,b,c,d;
    b = alpha*(-wx)*0.017;
    c = alpha*(-wy)*0.017;
    d = alpha*(-wz)*0.017;
    a = 1 - 0.5*(b*b+c*c+d*d);
    Quaternion result = quaternion_initialize(a,b,c,d);
    return result;
}
