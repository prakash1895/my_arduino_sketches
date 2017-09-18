

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define GPS_tx 3
#define GPS_rx 2

#define GSM_tx 9
#define GSM_rx 10

#define LED_pin 7
#define Buzzer_pin 6

#define x_min 10.766355
#define x_max 10.766944

#define y_min 78.816036
#define y_max 78.816889
  
SoftwareSerial GPS_serial(GPS_tx, GPS_rx);  //GPS Serial Communication
SoftwareSerial GSM_serial(GSM_tx, GSM_rx);  //GSM Serial Communication

Adafruit_GPS GPS(&GPS_serial);

#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean);

int GPS_stablizer = 0;
int flag_prev=0;  // flag_prev == 0 => Inside safe zone 
                  // flag_prev == 1 => Outside safe zone
void setup()  
{
  //Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  //Serial.println("Adafruit GPS library basic test!");
  
  pinMode(LED_pin, OUTPUT);
  pinMode(Buzzer_pin, OUTPUT);
  
  GSM_serial.begin(9600);   // Setting the baud rate of GSM Module  
  delay(100);
  SendMessage("GSM has turned ON and is working");
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  useInterrupt(true);
  delay(1000); 
}

SIGNAL(TIMER0_COMPA_vect) 
{
  char c = GPS.read();
  #ifdef UDR0
  if (GPSECHO)
      if (c) UDR0 = c;  
  #endif
}

void useInterrupt(boolean v)
{
  if (v) 
  {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else
  {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop()                  
{
  
  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA()))   
      return;  
  }
  
    delay(100);
 
    if (GPS.fix)
    { 
      GPS_stablizer++ ;
      if (GPS_stablizer > 150) 
          GPS_stablizer = 150;
      if (GPS_stablizer > 100)
        {
          checking(GPS.latitudeDegrees,GPS.longitudeDegrees);
        }
    }
    
    else
     {
       digitalWrite(Buzzer_pin,LOW);
       
       digitalWrite(LED_pin,HIGH);
       delay(75);
       digitalWrite(LED_pin,LOW);
       delay(75);
     }
}

 void SendMessage(String str)
{
 
  GSM_serial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  GSM_serial.println("AT+CMGS=\"+919444969493\"\r"); // Replace x with mobile number
  delay(1000);
  GSM_serial.println(str);// The SMS text you want to send
  delay(100);
  GSM_serial.println((char)26);// ASCII code of CTRL+Z
  delay(5000);

}


void checking(float lat,float lng)
{   
    int flag_lat = 1;
    int flag_long = 1;
    
    if (lat > x_min && lat < x_max)
        flag_lat = 1;
    else
        flag_lat = 0;

    if (lng > y_min && lng < y_max)
        flag_long = 1;
    else
        flag_long = 0; 	

if (flag_lat == 0 || flag_long == 0)
  {
     digitalWrite(LED_pin,HIGH);
     digitalWrite(Buzzer_pin,HIGH);
     delay(250);

     digitalWrite(LED_pin,LOW);
     digitalWrite(Buzzer_pin,LOW);
     delay(250);
     
  	if (flag_prev==0)
	  { 
	    flag_prev=1;
      String stri;
      char str1[10],str2[10];
      stri= "Latitude : " + String(dtostrf(lat,3,6,str1)) + "   Longitude : "+String(dtostrf(lng,3,6,str2));
	    SendMessage(stri);
    }
  }
  
    else
  	{
      flag_prev=0;
      digitalWrite(LED_pin,HIGH);
      digitalWrite(Buzzer_pin,LOW);
    }
	
}

