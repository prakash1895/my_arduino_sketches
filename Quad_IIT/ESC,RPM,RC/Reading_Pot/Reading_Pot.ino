
#define kp_input A0
double Kp_Roll = 0;

void setup()
{
  pinMode(kp_input,INPUT);
  Serial.begin(9600);
  Serial.flush();
 
}


  void loop()
 {
   Analog_Read();
   Serial.println(Kp_Roll);
 }
 
 
void Analog_Read()
{
  Kp_Roll = analogRead(kp_input);
  if (Kp_Roll > 1000) Kp_Roll = 1000;
  Kp_Roll = Kp_Roll*5*pow(10,-6);   
}
