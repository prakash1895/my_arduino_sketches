#include<SoftwareSerial.h>

SoftwareSerial Rhino_serial(8,9);

void setup() 
{   
  Serial.begin(9600);
  Rhino_serial.begin(9600);
  delay(10);

 
 Rhino_send("S255\n\r");
 Serial.println("");
 Serial.println(Rhino_read("P\n\r")); 
}

void Rhino_send(char s[])
{

  Rhino_serial.write(s);
  delay(10); 
  while(Rhino_serial.available())
  Rhino_serial.read();
  delay(10);
}

long int Rhino_read(char r[])
{
 String str;
 String Number = "";
 long int value = 0;

 Rhino_serial.write(r);
 delay(10);
  
  if (Rhino_serial.available() > 0)
  {
    str = Rhino_serial.readString();
  }

 for (int i=0; str[i]!='\0';i++)
   {
      if (isDigit(str[i]))
        Number += str[i]; 
   }
 value = Number.toInt();
 return value;
}

void loop() 
{
     
}
