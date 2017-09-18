
#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;
bool set_; 

geometry_msgs::Vector3 num;
ros::Publisher p("Arduino_vel", &num);

void messageCb(const geometry_msgs::Twist& msg)
{
  num.x = msg.linear.x;
  num.y = msg.linear.y;
  num.z = msg.angular.z;
  
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> s("/turtle1/cmd_vel",messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  Serial.begin(57600);
}

void loop()
{  
  p.publish(&num);
  nh.spinOnce();
  delay(10);
}

