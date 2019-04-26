#include<Wire.h>
#include <quaternion.h>
#include <sensor_processing_lib.h>
#include <vector_3d.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX=0,AcY=0,AcZ=1,Tmp,GyX=0,GyY=0,GyZ=0,count=0;
unsigned long Start = 0,loop_start,temp2;
float delta,wx,wy,wz;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc;

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
  Wire.write(0x08);     // set to 0x08 (+- 4g)
  Wire.endTransmission(true);
  Serial.begin(115200);
  Start = millis(); 
  fused_vector = vector_3d_initialize(0.0,0.0,-1.0);
  q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);
  loop_start = millis();
}
void loop(){
  
  wx = 0.0005323*GyX;
  wy = 0.0005323*GyY;
  wz = 0.0005323*GyZ;
  
  delta = 0.001*(millis()-Start);
  fused_vector = update_fused_vector(fused_vector,AcX,AcY,AcZ,wx,wy,wz,delta);
  
  q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
  angles = quaternion_to_euler_angles(q_acc);

  Serial.print("A ");
  Serial.print(int(angles.yaw));Serial.print(" ");
  Serial.print(int(angles.pitch));Serial.print(" ");
  Serial.println(int(angles.roll));

  Start = millis();

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

  /*count += 1;
  if (count>=500)
  {
    temp2 = millis()-loop_start;
    loop_start = millis();
    Serial.println(temp2);
    count = 0;
  }*/
}
