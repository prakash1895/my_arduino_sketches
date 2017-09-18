// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float mag, n_mx , n_my  , n_mz;
double mx2,my2,mz2;
float acc_ang_x,acc_ang_y,acc_ang_z;
float ypr[3] = {0.0f,0.0f,0.0f};

uint32_t dt_last;
float dt;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    dt_last = micros();
}

void loop() {
    
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  //Serial.print(ax); Serial.print("\t");
  //Serial.print(ay); Serial.print("\t");
  //Serial.print(az); Serial.println("\t");
  //Serial.print(gx); Serial.print("\t");
  //Serial.print(gy); Serial.print("\t");
  //Serial.print(gz); Serial.println("\t");
  
    
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.println("\t");
  delay(200);
  
    n_mx = mx/1000;
    n_my = my/1000;
    n_mz = mz/1000;
  
 
    mag = sqrt(n_mx*n_mx + n_my*n_my + n_mz*n_mz);
   
    n_mx = (n_mx/mag);
    n_my = (n_my/mag);
    n_mz = (n_mz/mag);
    
    ax = (ax*2/2048);
    ay = (ay*2/2048);
    az = (az*2/2048);
    
    acc_ang_x = atan2(ay, sqrt(ax*ax + az*az)) * 180/PI;
    acc_ang_y = atan2(ax, sqrt(ay*ay + az*az)) * 180/PI;
    
    dt = (micros() - dt_last)/1000000;
    dt_last = micros();
    
    ypr[2] = 0.93 * (ypr[2] + gx * dt) + 0.07 * acc_ang_x;
    ypr[1] = 0.93 * (ypr[1] + gy * dt) + 0.07 * acc_ang_y;
  
/*   Sketch github

    mx2 = n_mx*cos(ypr[1]) + n_my*sin(ypr[1])*sin(ypr[2]) + n_mz*sin(ypr[1])*cos(ypr[2]);
    my2 = n_mz*sin(ypr[2]) - n_my*cos(ypr[2]);
  
  */
 
   

    mx2 = n_mx*cos(ypr[1]) + n_mz*sin(ypr[1]);
    my2 = n_mx*sin(ypr[2])*sin(ypr[1]) + n_my*cos(ypr[2]) - n_mz*sin(ypr[2])*cos(ypr[1]);
    mz2 = -n_mx*cos(ypr[2])*sin(ypr[1]) + n_my*sin(ypr[2]) + n_mz*cos(ypr[2])*cos(ypr[1]);
  
   
  /*
  my2 = -n_my*cos(ypr[2]) + n_mz*sin(ypr[2]);
  mx2 =  n_mx*cos(ypr[1]) + n_my*sin(ypr[1])*sin(ypr[2]) + n_mz*sin(ypr[1])*cos(ypr[2]);
  */
    ypr[0] = atan2(my2,mx2)*180/PI;
    
    /*
   Serial.print(ypr[0]); Serial.print("\t"); 
   Serial.print(ypr[1]); Serial.print("\t");
   Serial.print(ypr[2]); Serial.println("\t");
    */
    
    //Serial.print(mx2); Serial.print("\t");
    //Serial.print(my2); Serial.println("\t");
    //const float N = 256;
    //float mag = mx*mx/N + my*my/N + mz*mz/N;

    //Serial.print(mag); Serial.print("\t");
    //for (int i=0; i<mag; i++)
        //Serial.print("*");
    //Serial.print("\n");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(50);
}
