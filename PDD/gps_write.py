#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

x1 = 10.4
y1 = 78.4

x2 = 10.5
y2 = 78.5


def callback(data):
    x_lati = data.x  
    y_longi = data.y

    flag_a = 1
    flag_b = 1
    
    if x_lati > x1 and x_lati < x2:
        flag_a = 1
    else:
        flag_a = 0

    if y_longi > y1 and y_longi < y2:
        flag_b = 1
    else:
        flag_b = 0

    if flag_a == 0 or flag_b == 0:
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(18, GPIO.HIGH)
        time.sleep(0.5)
        
        GPIO.output(23, GPIO.LOW)
        GPIO.output(18, GPIO.LOW)
        time.sleep(0.5)
    
    print "Latitude :"+str(x_lati)+"  Longitude :"+str(y_longi)
    
def listener():

    rospy.init_node('GPS_subscriber', anonymous=True)    
    rospy.Subscriber("gps_listener", Vector3, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
