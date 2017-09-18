#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

//Variables to store the Accelorometer and Gyro Values
float AcX,AcY,AcZ,GyX,GyY,GyZ;
float accel_angle_x,accel_angle_y; 

//variables to store the Yaw,Pitch and Roll Angle Values
float ypr[3] = {0.0f,0.0f,0.0f};
float yprLast[3] = {0.0f,0.0f,0.0f};

//Variable to store the time change 'dt'
uint32_t dt_last;
uint8_t i2cData[14];

void setup()
{
  initMPU(); // Run the Sensor initializing Function
 
  //initializing Serial Communication
  Serial.begin(9600); 
  Serial.flush();
  Serial.println("Setup Done");

}

void loop()
{
  getYPR(); //Run the Angle computing function 
}

//Function to Initialze the IMU Sensor

void initMPU()
{
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; 

  i2cData[0] = 7; 
  i2cData[1] = 0x00; 
  i2cData[2] = 0x00; 
  i2cData[3] = 0x00; 
  while (i2cWrite(0x19, i2cData, 4, false)); 
  while (i2cWrite(0x6B, 0x01, true)); 

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
  { 
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); 

  while (i2cRead(0x3B, i2cData, 6));
  AcX = (i2cData[0] << 8) | i2cData[1];
  AcY = (i2cData[2] << 8) | i2cData[3];
  AcZ = (i2cData[4] << 8) | i2cData[5];

  double accel_angle_x = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 180/PI ;
  double accel_angle_y = atan2(AcX, sqrt(AcY*AcY + AcZ*AcZ)) * 180/PI ;
  
  ypr[2] = accel_angle_x;
  ypr[1] = accel_angle_y;
  
  dt_last = micros();
}

void getYPR()
{
   
  while (i2cRead(0x3B, i2cData, 14));
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcY = ((i2cData[2] << 8) | i2cData[3]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  
  GyX = (i2cData[8] << 8) | i2cData[9];
  GyY = (i2cData[10] << 8) | i2cData[11];
  GyZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - dt_last) / 1000000; // Calculate delta time
  dt_last = micros();

// converting Acceleration to degree using inverse tan function
  double accel_angle_x = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 180/PI ;
  double accel_angle_y = atan2(-AcX, sqrt(AcY*AcY + AcZ*AcZ)) * 180/PI ;
  
  double GyX = GyX / 131.0; // Convert to deg/s
  double GyY = GyY / 131.0; // Convert to deg/s

   // Calculate the angle using a Complimentary filter
   
  ypr[2] = 0.93 * (ypr[2] + GyX * dt) + 0.07 * accel_angle_x;
  ypr[1] = 0.93 * (ypr[1] + GyY * dt) + 0.07 * accel_angle_y;


  /* Print Data */

  Serial.print("roll:  "); 
  Serial.print(ypr[2]+0.48); // adding some angle offset
  Serial.print("\tpitch:  "); 
  Serial.println(ypr[1]+2.15);   // adding some angle offset
  delay(1);                                   
 
}
