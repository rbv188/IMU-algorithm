#ifndef SENSOR_PROCESSING_INCLUDED
#define SENSOR_PROCESSING_INCLUDED

#include "quaternion.h"

Quaternion quaternion_from_accelerometer(float ax, float ay, float az);
Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time);

#endif // SENSOR_FUSION_LIB_H_INCLUDED
