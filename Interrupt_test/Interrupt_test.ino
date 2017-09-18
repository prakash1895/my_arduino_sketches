#define Distress_button 2


void setup() 
{
  
  pinMode(13, OUTPUT);
   
  pinMode(Distress_button, INPUT_PULLUP); //CHANGES
  //attachInterrupt(digitalPinToInterrupt(Distress_button),Distress, FALLING);

}

void loop() 
{
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);         
}


void Distress()
{
for(int i=0;i<15;i++)
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(250);         
}
}

