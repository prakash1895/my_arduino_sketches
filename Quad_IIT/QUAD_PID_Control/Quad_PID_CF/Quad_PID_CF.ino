#include <Servo.h>   // for servo motors, 12 servos in arduino and 48 servos in mega
#include <Wire.h>    //This library allows you to communicate with I2C/TWI devices.In UNO A4-SDA(data line) and A5-SCL(clock line)

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

//connections for the channels in the RC Receiver

byte RC_1 = A3;
byte RC_2 = 7; 
byte RC_3 = A2;
byte RC_4 = A1;

//ESC configuration

#define ESC_MIN 1000  // PWM VALUES TO SEND SIGNALS TO ESCs
#define ESC_MAX 2000

//RC Receiver Values MIN-MAX

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

// Pitch & Roll Min-Max Angles

#define PITCH_MIN -38   
#define PITCH_MAX 38
#define ROLL_MIN -38
#define ROLL_MAX 38

#define alpha 0.97  // Complementary Filter tuning paramaeter

//float Kd_Pitch = 0.0;   //uncomment this line and comment the corresponding "#define" statement above while tuning the PID gains
float P_Pitch_error,I_Pitch_error,D_Pitch_error;
float P_Roll_error,I_Roll_error,D_Roll_error;

float pitch_error,roll_error,Pitch_last_error,Roll_last_error;

unsigned long int now,lastTime;
float timeChange;

//IMU Sensor variables

const int MPU=0x68;  

float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float accel_angle_x,accel_angle_y; 

uint32_t dt_last;
uint8_t i2cData[14];

float ypr[3] = {0.0f,0.0f,0.0f};
float yprLast[3] = {0.0f,0.0f,0.0f};

float rc_roll,rc_pitch,rc_yaw,rc_throttle;         // RC channel inputs

int velocity;         // global velocity
float va, vb, vc, vd; // velocities of each motor
Servo a,b,c,d;        // ESCs variables

void setup()
{ 
   //pinMode(A0, INPUT);  //uncomment during tuning using Potemtiometer
  initESCs();                       
  initMPU();
  initRC();  
  initVar();
  lastTime = millis();
  dt_last = micros();
  
  Serial.begin(9600);                 
  Serial.flush();
  
}

void loop()
{   
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

/* Computing the Pitch Roll Angles using Complementary filter */

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
  
  double accel_angle_x = atan2(AcY, sqrt(AcX*AcX + AcZ*AcZ)) * 180/PI ;
  double accel_angle_y = atan2(-AcX, sqrt(AcY*AcY + AcZ*AcZ)) * 180/PI ;
  
  double GyX = GyX / 131.0; // Convert to deg/s
  double GyY = GyY / 131.0; // Convert to deg/s

 /*Calculate the angle using a Complimentary filter */
 
  ypr[2] = alpha * (ypr[2] + GyX * dt) + (1-alpha) * accel_angle_x; 
  ypr[1] = alpha * (ypr[1] + GyY * dt) + (1-alpha) * accel_angle_y;


  /* Print Roll and Pitch Angles */

  Serial.print("roll:  "); 
  Serial.print(ypr[2]+0.48); 
  Serial.print("\tpitch:  "); 
  Serial.println(ypr[1]+2.15);   
  delay(1);                                   
 
}

// uncomment the function below to tune the PID gains using Potentiomter

/*
void Analog_Read()
{
  Kd_Pitch = analogRead(A0);
  if (Kd_Pitch > 1000) Kd_Pitch = 1000;
  Kd_Pitch = Kd_Pitch*9*pow(10,-8);
    
}
*/

/* Implementing the PID Algorithm */

void calculateError()
{
  
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
      A,C Axis corresponds to Pitch Axis
      B,D Axis corresponds to Roll  Axis
  */
  
  a.writeMicroseconds(va); 
  c.writeMicroseconds(vc); 
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);  
 
}

