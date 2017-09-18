#include <SoftwareSerial.h>
 
#define SIM800_TX_PIN 9
#define SIM800_RX_PIN 10

#define Distress_button 2
 
//Create software serial object to communicate with SIM800
SoftwareSerial serialSIM800(SIM800_TX_PIN,SIM800_RX_PIN);
String lonlat;
 
void setup() {
   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    
  //Begin serial comunication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  while(!Serial);
   
  //Being serial communication witj Arduino and SIM800
  serialSIM800.begin(9600);
  delay(1000);
  
   pinMode(Distress_button, INPUT_PULLUP); //CHANGES
  attachInterrupt(digitalPinToInterrupt(Distress_button),Distress, FALLING);
   
  Serial.println("Setup Complete!");
  
  delay( 250 );
    while( !setupGPRS()){
      Serial.println("Failed to setup GPRS.");
      delay(100);
      Serial.println("Trying Again");
    }
//   delay( 250 );
//    while( !sendSMS("+917010254199","Hello")){
//      Serial.println("Failed to send SMS.");
//      delay(100);
//      Serial.println("Trying Again");
//    }  
  
}
 
void loop() {
  //Read SIM800 output (if available) and print it in Arduino IDE Serial Monitor
  if(serialSIM800.available()){
    Serial.write(serialSIM800.read());
  }
  //Read Arduino IDE Serial Monitor inputs (if available) and send them to SIM800
  if(Serial.available()){    
    serialSIM800.write(Serial.read());
  }
}

void cleanBuffer() 
{ 
delay( 250 ); 
while ( serialSIM800.available() > 0) 
{ serialSIM800.read(); 
// Clean the input buffer 
delay(50); 
} 
}

int8_t waitFor(const char* expected_answer1, const char* expected_answer2)
{
uint8_t x=0, answer=0;
char response[100];
unsigned long previous;

memset(response, (char)0, 100);    // Initialize the string

delay( 250 );

x = 0;
previous = millis();

// this loop waits for the answer
do{
    // if there are data in the UART input buffer, reads it and checks for the asnwer
    if(serialSIM800.available() > 0){
        response[x] = serialSIM800.read();
        x++;
        // check if the desired answer 1  is in the response of the module
        if (strstr(response, expected_answer1) != NULL)
        {
            answer = 1;
        }
        // check if the desired answer 2 is in the response of the module
        else if (strstr(response, expected_answer2) != NULL)
        {
            answer = 2;
        }
    }
    delay(10);
}
// Waits for the asnwer with time out
while((answer == 0) && ((millis() - previous) < 10000 ));

return answer;
}


bool sendSMS(const char* number, const char* text)
{
    cleanBuffer();
    serialSIM800.println("AT+CMGF=1");
    if ( waitFor("OK", "ERROR") != 1 ) return false;

    cleanBuffer();
    serialSIM800.print("AT+CMGS=\"");
    serialSIM800.print(number);
    serialSIM800.println("\"");
    if ( waitFor(">", "ERROR") != 1 ) return false;

    cleanBuffer();
    serialSIM800.print(text);
    serialSIM800.println((char)26);
    if ( waitFor("+CMGS:", "ERROR") != 1 ) return false;
    delay(2000);
    return true;
}


bool setupGPRS()
{
    delay( 250 );
    if ( !setConnectionType() ){
      Serial.println("Failed to set connection type to GPRS.");
      return false;
    }
    delay( 250 );
    if ( !setAPN("www") ){
      Serial.println("Failed to setup APN.");
      return false;
    }
    delay( 250 );
    if ( !startGPRS() ){
      Serial.println("Failed to start GPRS.");
      //return false;
    }
    delay( 250 );
    if ( !hasIP() ){
      Serial.println("Failed to acquire IP address.");
     // return false;
    }
     delay( 250 );
    if ( !getloc() ){
      Serial.println("Failed to acquire Location.");
       return false;
    }

    return true;
}

bool setConnectionType()
{
    cleanBuffer();
    serialSIM800.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    if ( waitFor("OK", "ERROR") != 1 ) return false;

    return true;
}

bool setAPN(const char* apn)
{
    cleanBuffer();
    serialSIM800.print("AT+SAPBR=3,1,\"APN\",\"");
    serialSIM800.print(apn);
    serialSIM800.println("\"");
    if ( waitFor("OK", "ERROR") != 1 ) return false;

    return true;
}

bool startGPRS()
{
    cleanBuffer();
    serialSIM800.println("AT+SAPBR=1,1");
    if ( waitFor("OK", "ERROR") != 1 ) return false;

    return true;
}

bool hasIP()
{
    cleanBuffer();
    serialSIM800.println("AT+SAPBR=2,1");
    if ( waitFor("0.0.0.0", "OK") != 2 ) return false;

    return true;
}

bool getloc()
{
  cleanBuffer();
  serialSIM800.println("AT+CIPGSMLOC=1,1");
  if (waitFor2("++CIPGSMLOC: 601", "OK") !=2) return false;
  
  return true;
}


int8_t waitFor2(const char* expected_answer1, const char* expected_answer2)
{
uint8_t x=0, answer=0;
char response[500];
unsigned long previous;

memset(response, (char)0, 100);    // Initialize the string

delay( 250 );

x = 0;
previous = millis();

// this loop waits for the answer
do{
    // if there are data in the UART input buffer, reads it and checks for the asnwer
    if(serialSIM800.available() > 0){
        response[x] = serialSIM800.read();
        x++;
        // check if the desired answer 1  is in the response of the module
        if (strstr(response, expected_answer1) != NULL)
        {
            answer = 1;
            
        }
        // check if the desired answer 2 is in the response of the module
        else if (strstr(response, expected_answer2) != NULL)
        {
            answer = 2;
        }
    }
    delay(10);
}
// Waits for the asnwer with time out
while((answer == 0) && ((millis() - previous) < 10000 ));

if (answer==2)
{
String m(response);
char* m1 =strstr(response,"0,");
int postion = m1-response;
Serial.println(m.substring(postion+2,postion+21));
lonlat=m.substring(postion+2,postion+21);

}

return answer;
}


void Distress()
{

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);         
  
  char* message;
  int str_len = lonlat.length()+1;
  lonlat.toCharArray(message,str_len); 
  
  char* emer;
  emer = "Emergency";
  
  strcat(message,emer);
  Serial.println(message);
  sendSMS("+917010254199",message);
 
}

  
