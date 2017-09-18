

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


MPU6050 mpu;                           // mpu interface object

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0000f,0.0000f,0.0000f};       // yaw pitch roll values
float yprLast[3] = {0.0000f, 0.0000f, 0.0000f};

volatile bool mpuInterrupt = false;    //interrupt flag

uint8_t flag = 0;
float yaw_offset,pitch_offset,roll_offset; 

void setup()
{                                      
  Serial.begin(9600);                
  Serial.flush();
  initMPU();
  Serial.println("setup done");
  
  
do
{
   
  for(int i=0;i<100;i++)
    getYPR();
   
    
  if ((abs(ypr[0] - yprLast[0]) > 0.1)) // && (abs(ypr[1] - yprLast[1]) > 0.3) && (abs(ypr[2] - yprLast[2]) > 0.3))
    flag = 0;
  else
     flag = 1; 
  Serial.println("Calibrating...");
  
  yprLast[0] = ypr[0];    // updating yaw pitch roll value
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  
} while(flag==0);

yaw_offset = ypr[0];
pitch_offset = ypr[1];
roll_offset = ypr[2];

Serial.println("Calibration DONE :)");
          Serial.print("offset\t");
          Serial.print(yaw_offset);
          Serial.print("\t");
          Serial.print(pitch_offset);
          Serial.print("\t");
          Serial.println(roll_offset); 
}

/* loop function
 *
 */

void loop()
{ 
  while(!mpuInterrupt && fifoCount < packetSize)
  {  
    //Do nothing while MPU is not working     
  }

  getYPR();  
  
  if(abs(ypr[0]-yprLast[0])>20) ypr[0] = yprLast[0];   // difference more than 20
  if(abs(ypr[1]-yprLast[1])>20) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>20) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];    // updating yaw pitch roll value
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  
  
    /* print Yaw Pitch Roll Angles */
          Serial.print("ypr\t");
          Serial.print(ypr[0]-yaw_offset);
          Serial.print("\t");
          Serial.print(ypr[1]-pitch_offset);
          Serial.print("\t");
          Serial.println(ypr[2]-roll_offset);  
  
}


/* Initializing the IMU Sensor */

void initMPU()
{
  Serial.println("INIT MPU");
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw using built-in Libraries
 */

void getYPR()    
{ 
  
    mpuInterrupt = false;             // seeing if the mpu is ready
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024)
    { 
     // reset on buffer overflow   
      mpu.resetFIFO(); 
    
    }
    
    else if(mpuIntStatus & 0x02)   
    {
    
      while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize; 
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  
       
       
  ypr[0] = ypr[0] * 180/M_PI;     // converting radians to degrees
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;    
   delay(200);
   
  }     
}

void dmpDataReady() 
{
    mpuInterrupt = true;
}
