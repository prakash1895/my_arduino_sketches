/*
          RC Min Max RANGE

Trim in lowest position for all channels.

Roll    : 1000 to 1830
Pitch   : 1000 to 1750
Throttle: 1000 to 1870
Yaw     : 1090 to 1910

*/

#include <Servo.h> 
 
 // ESC varaiables
 
Servo esc_A;
Servo esc_B;
Servo esc_C;
Servo esc_D;

//RC_receiver- Arduino Pin Configuration
byte RC_RUD_ROLL= A3; 
byte RC_AIL_PITCH = 7; 
byte RC_THR = A2; 
byte RC_ELE_YAW = A1;

//RC variables
int rc_thr = 0;
int rc_thr_map = 0;

void setup() 
{  
  
  //initializing the RC Receiver 
  pinMode(RC_RUD_ROLL, INPUT);
  pinMode(RC_AIL_PITCH, INPUT);
  pinMode(RC_THR, INPUT);
  pinMode(RC_ELE_YAW, INPUT);

//initializing Serial Communication
  Serial.begin(9600);
  Serial.flush();

//initializing the ESC PWM pins
  esc_C.attach(3);
  esc_D.attach(9);
  esc_A.attach(10);
  esc_B.attach(11);
  
  delay(100);   
}

void loop() 
{
  rc_thr = pulseIn(RC_THR, HIGH); //Reading the RC input value
  rc_thr_map = map(rc_thr, 1310, 2150, 1000, 2000); // Mapping it to PWM signal
  
  //limiting the Signal 
  if (rc_thr_map < 1200)
      rc_thr_map = 1000;
  if (rc_thr_map > 1800)
      rc_thr_map = 2000;

//uncomment to print the RC values
 
 /* 
  Serial.print("rc_value:   ");
  Serial.println(rc_thr);
  
  Serial.print("\tMicroSeconds Value:   ");
  Serial.println(rc_thr_map);
  */   
 
 //Sending the PWM signal to the ESCs  
 esc_A.writeMicroseconds(rc_thr_map);
 esc_B.writeMicroseconds(rc_thr_map);
 esc_C.writeMicroseconds(rc_thr_map);
 esc_D.writeMicroseconds(rc_thr_map);
 
  }
