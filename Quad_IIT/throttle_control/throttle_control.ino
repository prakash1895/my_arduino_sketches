/*
          RC Min Max RANGE

Trim in lowest position for all channels.

Roll    : 1000 to 1830
Pitch   : 1000 to 1750
Throttle: 1000 to 1870
Yaw     : 1090 to 1910

*/

#include <Servo.h> 
 
 // setting up ESC variables
 
Servo esc_A;
Servo esc_B;
Servo esc_C;
Servo esc_D;

//RC Receiver - Arduino Pin configuration

byte RC_RUD_ROLL= A3; 
byte RC_AIL_PITCH = 7; 
byte RC_THR = A2; 
byte RC_ELE_YAW = A1;

// RC varaibles for storing the input values
int rc_thr = 0;
int rc_thr_map = 0;

void setup() 
{  
  
  // Setting the RC pins as Input to the Arduino
  pinMode(RC_RUD_ROLL, INPUT);
  pinMode(RC_AIL_PITCH, INPUT);
  pinMode(RC_THR, INPUT);
  pinMode(RC_ELE_YAW, INPUT);

// Initializing Serial Communication
  Serial.begin(9600);
  Serial.flush();

// ESC - Arduino Pin Configuration
  esc_C.attach(3);
  esc_D.attach(9);
  esc_A.attach(10);
  esc_B.attach(11);
  
  delay(100);
  
  // Armimg the ESCs
  esc_A.writeMicroseconds(900);
  esc_B.writeMicroseconds(900);
  esc_C.writeMicroseconds(900);
  esc_D.writeMicroseconds(900);
  delay(5000);
    
}

void loop() 
{
  // Reading the RC input Values
  rc_thr = pulseIn(RC_THR, HIGH);
  
  //Mapping the RC values to Signal
  rc_thr_map = map(rc_thr, 1310, 2150, 1000, 2000);
  
  if (rc_thr_map < 1000)
      rc_thr_map = 1000;
  if (rc_thr_map > 2000)
      rc_thr_map = 2000;

//uncomment to print the RC values
 
 /* 
  Serial.print("rc_value:   ");
  Serial.print(rc_thr);
  
  Serial.print("\tMicroSeconds Value:   ");
  Serial.println(rc_thr_map);
  */    
  
 // Sending PWM signals to the ESCs
 
 esc_A.writeMicroseconds(rc_thr_map);
 esc_B.writeMicroseconds(rc_thr_map);
 esc_C.writeMicroseconds(rc_thr_map);
 esc_D.writeMicroseconds(rc_thr_map);
  
  }
