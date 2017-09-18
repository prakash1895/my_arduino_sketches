
#include <Wire.h>
#include "Kalman.h" 
#define RESTRICT_PITCH 

//creating Kalman Objects
Kalman kalmanX; 
Kalman kalmanY;

/* Variables to store IMU Sensor Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
float ypr[3] = {0.0f,0.0f,0.0f};  

//Variable to store the time change 'dt'
uint32_t timer;
uint8_t i2cData[14]; 

void setup()
{
  initMPU();
  Serial.begin(9600);
}

void loop()
{
  getYPR();
}

//Function to initialize the IMU Sensor
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

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  
#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  timer = micros();
}

//Function to compute the Angles using Kalman Filter
void getYPR() 
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; 
  timer = micros();


#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; 
  double gyroYrate = gyroY / 131.0; 

#ifdef RESTRICT_PITCH
  if ((roll < -90 && ypr[2] > 90) || (roll > 90 && ypr[2] < -90)) 
  {
    kalmanX.setAngle(roll);
    ypr[2] = roll;
  } 
  else
    ypr[2] = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(ypr[2]) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    ypr[1] = kalmanY.getAngle(pitch, gyroYrate, dt);
#else

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && ypr[1] > 90) || (pitch > 90 && ypr[1] < -90)) 
  {
    kalmanY.setAngle(pitch);
    ypr[1] = pitch; 
  } 
  else
    ypr[1] = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(ypr[1]) > 90)
    gyroXrate = -gyroXrate; 
    ypr[2] = kalmanX.getAngle(roll, gyroXrate, dt); 
#endif

/* print Data */

  Serial.print("roll:  "); 
  Serial.print(ypr[2]+0.47); 
  Serial.print("\tpitch:  ");
  Serial.println(ypr[1]+2.1); 
  
}
