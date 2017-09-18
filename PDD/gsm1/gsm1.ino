#include <SoftwareSerial.h>

#define GSM_tx 9
#define GSM_rx 10

SoftwareSerial GSM_serial(GSM_tx, GSM_rx);

void setup()
{
  GSM_serial.begin(9600);   // Setting the baud rate of GSM Module  
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  delay(100);
  SendMessage("The GSM is working fine");
}


void loop()
{
  if (Serial.available()>0)
    { 
   switch(Serial.read())
  {
    case 's':
      checking(11.000,72.000);
      break;
    case 'r':
      checking(22.000,82.000);
      break;
  }

 if (GSM_serial.available()>0)
   Serial.write(GSM_serial.read());
}
}


 void SendMessage(String str)
{
  GSM_serial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  GSM_serial.println("AT+CMGS=\"+917010254199\"\r"); // Replace x with mobile number
  delay(1000);
  GSM_serial.println(str);// The SMS text you want to send
  delay(100);
   GSM_serial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}


 int flag1=0;
 
 void checking(float lat,float lng)
{
    float x1,x2,y1,y2;
    x1 = 10;
    y1 = 70;

    x2 = 20;
    y2 = 80;
    
    int flag_a = 1;
    int flag_b = 1;
    
    if (lat > x1 && lat < x2)
        flag_a = 1;
    else
        flag_a = 0;

    if (lng > y1 && lng < y2)
        flag_b = 1;
    else
        flag_b = 0; 	

if (flag_a == 0 || flag_b == 0)
{
  	if (flag1==0)
	  { flag1=1;
            String stri;
            char str1[10],str2[10];
            stri= "Latitude : " + String(dtostrf(lat,3,6,str1)) + " \nLongitude : "+String(dtostrf(lng,3,6,str2));
	   SendMessage(stri);
          }
}
else
	{
          flag1=0;
        }
	
}
