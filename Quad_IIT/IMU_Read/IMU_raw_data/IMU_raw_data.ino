#include<Wire.h>

const int MPU=0x68; 

//variables to Store the IMU Sensor data

float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float accel_angle_x,accel_angle_y;

void setup()
{
  initMPU(); 
  Serial.begin(115200); 
  Serial.flush();
  Serial.println("Setup Done");
 
}

void loop()
{
  getYPR();
  //Serial.print("Roll:   ");
  //Serial.print(accel_angle_x);
  //Serial.print("\tPitch:   ");
  //Serial.println(accel_angle_y);
}

/* Initializing the IMU Sensor */

void initMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission(true); 
}

/* Computing the Pitch and Roll Angles without using any kind filter */

void getYPR()
{
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true); 
  
  // Reading the Acc. and gyro values from the Sensor
  AcX=Wire.read()<<8|Wire.read();  
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read(); 
  GyZ=Wire.read()<<8|Wire.read();  
  
  //Serial.print(AcX); Serial.print("\t");
  //Serial.print(AcY); Serial.print("\t");
  //Serial.print(AcZ); Serial.println("\t");
  Serial.print(GyX); Serial.print("\t");
  Serial.print(GyY); Serial.print("\t");
  Serial.print(GyZ); Serial.println("\t");
  
  AcX = AcX * 2/2048;
  AcY = AcY * 2/2048;
  AcZ = AcZ * 2/2048;
  
  // Converting Acceleration into Angles using Inverse tan function
  accel_angle_x = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 180/PI ;
  accel_angle_y = atan2(AcX, sqrt(AcY*AcY + AcZ*AcZ)) * 180/PI ;
 
}
