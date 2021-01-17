#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>


MPU6050 gyro(Wire);
Servo rightProp;
Servo leftProp;

float Input;
double Output;
float Setpoint;
double Kp=2,Ki=0.005,Kd=1.8;
double currentSignalRight=1000,currentSignalLeft=1000;
double error;
double lasterror;
double totalerror;
double deltaerror;
double time,timePrev,elapsedTime;

int max_control = 2000;
int min_control = 1000;

double SignalLeft;
double SignalRight;

void setup() {
  
  Serial.begin(9600);
  gyro.begin();
  gyro.calcGyroOffsets(true);
  rightProp.attach(9);
  leftProp.attach(10);
  int beginrl;
  
  for (beginrl = 1000; beginrl != 2000; beginrl++)
  {
     rightProp.writeMicroseconds(beginrl);
     leftProp.writeMicroseconds(beginrl);
     
  }
  rightProp.writeMicroseconds(1000);
  leftProp.writeMicroseconds(1000);
  delay(5000);
  
}

void pid(float &Input,double &Output,float Setpoint,double Kp,double Ki,double Kd )
{    
  
  timePrev = time; 
  time = millis(); 
  elapsedTime = (time - timePrev) / 1000;
  lasterror = error;
  error = Setpoint - Input;
  totalerror += error;
  deltaerror = error - lasterror;
  Output = Kp*error + (Ki*elapsedTime)*totalerror + (Kd/elapsedTime)*deltaerror;
  if (Output >= 900) Output = 900;
  SignalRight = 1300 + Output;
  SignalLeft =  1300 - Output;
  if (SignalRight > max_control) SignalRight = max_control;
  if (SignalLeft > max_control) SignalLeft = max_control;
  if (SignalRight < min_control) SignalRight = min_control;
  if (SignalLeft < min_control) SignalLeft = min_control;
  
}

void loop() {
  
  gyro.update();
  Input = gyro.getAngleY();
  Setpoint = 0;
  pid(Input,Output,Setpoint,Kp,Ki,Kd);

  rightProp.write(SignalRight);
  leftProp.write(SignalLeft);

}
