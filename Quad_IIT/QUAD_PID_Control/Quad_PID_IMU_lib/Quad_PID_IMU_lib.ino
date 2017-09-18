#include <Servo.h>   // for servo motors, 12 servos in arduino and 48 servos in mega
#include <Wire.h>    //This library allows you to communicate with I2C / TWI devices.In UNO A4-SDA(data line) and A5-SCL(clock line)
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include "MPU6050_6Axis_MotionApps20.h"

//////////// Arduino Pin configuration////////////


/*

**IMU Sensor Pin Configuration**

SENSOR      ARDUINO PIN

SDA          A4
SCL          A5
VCC          3.3v
GND          GND  
INT          2

**RC Receiver Pin Configuration**

RC RECEIVER                ARDUINO PIN

THR (Throttle Control)      A2
AIL (Pitch    Control)      7
RUD (Roll     Control)      A3
ELE (Yaw      Control)      A1

*/

// All are PWM pins for the ESCs
#define ESC_A 10
#define ESC_B 11
#define ESC_C 3
#define ESC_D 9

//connections for the channels in the RC controller
byte RC_1 = A3;
byte RC_2 = 7; 
byte RC_3 = A2;
byte RC_4 = A1;

//ESC configuration

#define ESC_MIN 1000  // PWM VALUES TO SEND SIGNALS TO ESCs
#define ESC_MAX 2000

//RC Receiver values MIN-MAX Range

#define RC_HIGH_CH1 2400
#define RC_LOW_CH1 1320

#define RC_HIGH_CH2 2290
#define RC_LOW_CH2 1310

#define RC_HIGH_CH3 2440
#define RC_LOW_CH3 1310

#define RC_HIGH_CH4 2490
#define RC_LOW_CH4 1420

// PID gains

#define Kp_Pitch 0.001248
#define Ki_Pitch 0.00018
#define Kd_Pitch 0.00003060

#define Kp_Roll 0.00055
#define Ki_Roll 0.00009
#define Kd_Roll 0.0000055

#define PITCH_MIN -38   //The angles in degree
#define PITCH_MAX 38
#define ROLL_MIN -38
#define ROLL_MAX 38

//float Kp_Pitch = 0.0;   //uncomment this line and comment the corresponding "#define" statement above while tuning the PID gains

float P_Pitch_error,I_Pitch_error,D_Pitch_error;
float P_Roll_error,I_Roll_error,D_Roll_error;

float pitch_error,roll_error,Pitch_last_error,Roll_last_error;

unsigned long int now,lastTime;
float timeChange;

//IMU Sensor variables

MPU6050 mpu;                           // mpu interface object
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

float rc_roll,rc_pitch,rc_yaw,rc_throttle;         // RC channel inputs

int velocity;          // global velocity
float va, vb, vc, vd;  // velocities
Servo a,b,c,d;         // ESC variables

void setup()
{ 
  //pinMode(A0, INPUT);  //uncomment this line while tuning using Potemtiometer
  Serial.begin(9600);                
  Serial.flush();
  
  initESCs();                       
  initMPU();
  initRC();  
  initVar();
  
  lastTime = millis(); 
}

void loop()
{  
 
  while(!mpuInterrupt && fifoCount < packetSize)
  {     
 
  } 
  getYPR();
  //Analog_Read();   //Used for tuning the PID gains uncomment duirng tuning
  calculateError();
  calculateVelocities();
  updateMotors();
  delay(25);                // for 100ms Sampling Time
}

/* Function to initilaize RC Receiver */

void initRC()
{
 pinMode(RC_1, INPUT);
 pinMode(RC_2, INPUT);
 pinMode(RC_3, INPUT);
 pinMode(RC_4, INPUT);
 
}

/* Function to initilaize IMU Sensor */

void initMPU()
{
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}
 
void dmpDataReady() 
{
    mpuInterrupt = true;
}

 /* Function to initilaize the ESCs */
 
void initESCs()
{
  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D); 
  delay(100);
  
  a.writeMicroseconds(900);
  b.writeMicroseconds(900);
  c.writeMicroseconds(900);
  d.writeMicroseconds(900);  
  delay(5000);  
  
}

void initVar()
{ 
  roll_error = 0;
  pitch_error = 0;
  va = 0;
  vb = 0;
  vc = 0;
  vd = 0;
  lastTime = 0;
  
}  

  /* Computing the Pitch Roll Angles using In-Built Libraries */
  
void getYPR()    
{ 
    mpuInterrupt = false;                                         
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024)
    { 
           
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
      
      // Converting Radians to Deg
      
  ypr[0] = (ypr[0] * 180/M_PI);     
  ypr[1] = (ypr[1] * 180/M_PI);
  ypr[2] = (ypr[2] * 180/M_PI);
  
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];  
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  
  yprLast[0] = ypr[0];    
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  
  /* Printing YAW PITCH ROLL Angles */
  
  Serial.print("yaw : ");
  Serial.print(ypr[0]);
  Serial.print("\tpitch:  ");
  Serial.print(ypr[1]);
  Serial.print("\troll:  ");
  Serial.println(ypr[2]);
  
}

  rc_roll = pulseIn(RC_1, HIGH);
  rc_pitch = pulseIn(RC_2, HIGH);
  
  rc_pitch = map(rc_pitch, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX); 
  rc_roll = map(rc_roll, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
   
  //  Pitch and Roll RC_Input Calibration
   
  if(rc_roll >= -8 && rc_roll <= 8)     rc_roll = 0;
  else if(rc_roll < -8)                 rc_roll = rc_roll + 8;
  else if(rc_roll > 8)                  rc_roll = rc_roll - 8;
  
  if(rc_pitch >= -12 && rc_pitch <= 4)  rc_pitch = 0;
  else if(rc_pitch < -12)               rc_pitch = rc_pitch + 8;
  else if(rc_pitch > 4)                 rc_pitch = rc_pitch - 8;
     
  rc_pitch = floor(rc_pitch/4);
  rc_pitch = rc_pitch*4;
  
  rc_roll = floor(rc_roll/4);
  rc_roll = rc_roll*4;
  
}

// execute the below unction only during PID tuning using Potentiometer
/*
void Analog_Read()
{
  Kp_Pitch = analogRead(A0);
  if (Kp_Pitch > 1000) Kp_Pitch = 1000;
  Kp_Pitch = Kp_Pitch*9*pow(10,-8);
    
}
*/

/* Implementing the PID Algorithm */

void calculateError()
{
  // calculating the dt change
  
   now = millis();
   timeChange = (float)(now - lastTime);
   timeChange = timeChange*pow(10,-3);
   
    //calcualting the error
    
   P_Pitch_error = rc_pitch - ypr[1];                             // Propotional error
   I_Pitch_error += (P_Pitch_error*timeChange);                   // Integral of error
   D_Pitch_error = (P_Pitch_error - Pitch_last_error)/timeChange; // Derivative of error
   
   pitch_error = Kp_Pitch*P_Pitch_error + Ki_Pitch*I_Pitch_error + Kd_Pitch*D_Pitch_error;  //controller output
  
   P_Roll_error = rc_roll - ypr[2];
   I_Roll_error += (P_Roll_error*timeChange);
   D_Roll_error = (P_Roll_error - Roll_last_error)/timeChange;
   
   roll_error = Kp_Roll*P_Roll_error + Ki_Roll*I_Roll_error + Kd_Roll*D_Roll_error;
   
    // updating the previous values
    
   Pitch_last_error = P_Pitch_error;
   Roll_last_error = P_Roll_error;
   lastTime = now;

}
  
   /* Calculating the Signal based on the PID Algorithm */
   
void calculateVelocities()
{   
  rc_throttle = pulseIn(RC_3,HIGH);
  velocity = map(rc_throttle,RC_LOW_CH3,RC_HIGH_CH3,ESC_MIN,ESC_MAX);  // ch3 for the throttle
  
  velocity = floor(velocity/50);
  velocity = velocity*50;
    
  va  = (1 + pitch_error)*velocity;
  vc  = (1 - pitch_error)*velocity;
  
  vb  = (1 + roll_error)*velocity;
  vd  = (1 - roll_error)*velocity;
  
  /*Limiting the Motor Signal if at all it goes out of range*/
 
  if (velocity < 1000)   
  {
       va=900;
       vb=900;
       vc=900;
       vd=900;
   }
     
 if (velocity > 2000)   
     {
       va=2000;
       vb=2000;
       vc=2000;
       vd=2000;
     }

}

/* Function to Send the signals to the motors */

void updateMotors()
{
   /*
      A,C Axis corresponds to Pitch  
      B,D Axis corresponds to Roll
  */
 
  a.writeMicroseconds(va); 
  c.writeMicroseconds(vc); 
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);  
 
}

