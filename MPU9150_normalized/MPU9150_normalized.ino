// Baud rate was changed from 57600 to 9600
//spinOnce had no effect Just include it once
// while printing Ax,Ay values we got only 0 or 9.8 no values in between


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;
geometry_msgs::PoseArray imu;
geometry_msgs::Vector3 acc;

//ros::Publisher IMU_pub("arduino_imu", &imu);
ros::Publisher accel("/arduino_acc", &acc);

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float Ax,Ay,Az;
float Gx,Gy,Gz;

#define LED_PIN 13
bool blinkState = false;

void setup()
{ Serial.begin(9600);
  nh.getHardware()->setBaud(9600);
  init_IMU(); 
  nh.initNode();
  //nh.advertise(IMU_pub); 
  nh.advertise(accel);
}

void loop()
{
  getYPR();
  //IMU_pub.publish(&imu);
  accel.publish(&acc);
  nh.spinOnce();
  //nh.spinOnce();
  delay(10);
  
}
void init_IMU() 
{
    
    Wire.begin();
    accelgyro.initialize();
    pinMode(LED_PIN, OUTPUT);
    
}

void getYPR()
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
   
    Ax = 9.81*ax/16384.0;
    Ay = (9.81*ay)/16384;
    Az = (9.81*az)/16384;
    
    Serial.print(Ax);Serial.print("\t");Serial.print(Ay);Serial.print("\t");Serial.println(Az);
    Gx = (gx*3.14)/(131*180);
    Gy = (gy*3.14)/(131*180);
    Gz = (gz*3.14)/(131*180);


  // acc is float64 type ax is int_16
  acc.x = Ax;
  acc.y = Ay;
  acc.z = Az;

  
   /*
  imu.poses[0].position.x = ax;
  imu.poses[0].position.y = ay;
  imu.poses[0].position.z = az;
  
  imu.poses[1].position.x = gx;
  imu.poses[1].position.y = gy;
  imu.poses[1].position.z = gz;
  
  imu.poses[2].position.x = mx;
  imu.poses[2].position.x = my;
  imu.poses[2].position.x = mz;
  */
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
}
