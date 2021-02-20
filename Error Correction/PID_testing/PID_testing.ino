#include <PID_v1.h>
double Setpoint, Input, Output;


PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup()
{
  Setpoint = 30;
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = 0;
  myPID.Compute();
  Serial.println(Output);
}
