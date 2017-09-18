#include<ros.h>
#include"std_msgs/Int16.h"

ros::NodeHandle n;
std_msgs::Int16 num;
ros::Publisher chat("arduino_talker",&num);

bool toggle= true;
int i =0 ;

void setup()
{
  n.initNode();
  n.advertise(chat);
  Serial.begin(57600);
}

void loop()
{
  toggle = !toggle;
  
  if(toggle)
      i = 1; 
  else 
      i = 0;
      
  num.data = i;
  chat.publish(&num);
  delay(500);    
  n.spinOnce();
}


  
  
