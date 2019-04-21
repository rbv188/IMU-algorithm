#include<Wire.h>
#include <quaternion.h>
#include <sensor_processing_lib.h>
#include <vector_3d.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX=0,AcY=0,AcZ=1,Tmp,GyX=0,GyY=0,GyZ=0;
unsigned long Start = 0;
float delta,ax,ay,az,wx,wy,wz;
euler_angles angles;
Quaternion q, q_new, q_acc, q_gyro;
vector_ijk virtual_gravity,sensor_gravity,temp;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x10);     // set to 0x10 (+- 1000 deg/s)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x08);     // set to 0x10 (+- 4g)
  Wire.endTransmission(true);
  Serial.begin(115200);
  Start = millis(); 
  q_gyro = quaternion_initialize(1.0,0.0,0.0,0.0);
  virtual_gravity = vector_3d_initialize(0.0,0.0,-1.0);
  q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);
}
void loop(){
  wx = 0.03*GyX;
  wy = 0.03*GyY;
  wz = 0.03*GyZ;
  ax = float(AcX);
  ay = float(AcY);
  az = float(AcZ);

  sensor_gravity.a = ax;
  sensor_gravity.b = ay;
  sensor_gravity.c = az;

  sensor_gravity = vector_3d_normalize(sensor_gravity);
  
  delta = 0.001*(millis()-Start);
  q_gyro = quaternion_from_gyro(wx,wy,wz,delta);
  virtual_gravity = quaternion_rotate_vector(temp,q_gyro);
  virtual_gravity = vector_3d_scale(virtual_gravity,15);
  temp = vector_3d_sum(virtual_gravity,sensor_gravity);
  temp = vector_3d_normalize(temp);
  q_acc = quaternion_from_accelerometer(temp.a,temp.b,temp.c);
  angles = quaternion_to_euler_angles(q_acc);

  Start = millis();

  Serial.print("A ");
  Serial.print(int(angles.yaw));Serial.print(" ");
  Serial.print(int(angles.pitch));Serial.print(" ");
  Serial.println(int(angles.roll));

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyX -= 8;
  GyY += 30;
  GyZ -= 10;
}
