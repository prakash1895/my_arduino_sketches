#include <Servo.h> 

/*
          RC Min Max RANGE

Trim in lowest position for all channels.

Roll    : 1000 to 1830
Pitch   : 1000 to 1735
Throttle: 1000 to 1865
Yaw     : 1090 to 1910

*/

//RC_receiver- Arduino Pin Configuration
byte RC_AIL_PITCH = 7; 
byte RC_ELE_YAW = A1;
byte RC_THR = A2; 
byte RC_RUD_ROLL= A3; 

//RC variables
int rc_roll = 0;
int rc_pitch = 0;
int rc_thr = 0;
int rc_yaw = 0;

void setup() 
{  
  
   // initializing the RC Receiver 
  pinMode(RC_RUD_ROLL, INPUT);
  pinMode(RC_AIL_PITCH, INPUT);
  pinMode(RC_THR, INPUT);
  pinMode(RC_ELE_YAW, INPUT);
  
  //initializing Serial Communication
  Serial.begin(9600);
  Serial.flush();
}

void loop() 
{
  //Reading the RC input value
  
  rc_roll = pulseIn(RC_RUD_ROLL, HIGH);
  rc_pitch = pulseIn(RC_AIL_PITCH, HIGH);
  rc_thr = pulseIn(RC_THR, HIGH);
  rc_yaw = pulseIn(RC_ELE_YAW, HIGH);
  
  //printing the RC input values
  Serial.print("Roll :");
  Serial.print(rc_roll);
  Serial.print("\tPitch :");
  Serial.print(rc_pitch);
  Serial.print("\tThrottle :");
  Serial.print(rc_thr);
  Serial.print("\tYaw :");
  Serial.println(rc_yaw);
  delay(500);
}
