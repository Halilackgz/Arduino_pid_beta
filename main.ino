#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>


MPU6050 gyro(Wire);
Servo rightProp;
Servo leftProp;
double Input;
double Output;
double Setpoint;
double Kp=2,Ki=0.01,Kd=8;
double currentSignalRight=1100,currentSignalLeft=1000;
double error;
double lasterror;
double totalerror;
double deltaerror;
double time,timePrev,elapsedTime;

int max_control = 2000;
int min_control = 1000;

double SignalLeft=1300;
double SignalRight=1300 ;

void setup() {
  Serial.begin(9600);
  gyro.begin();
  gyro.calcGyroOffsets(true);
  rightProp.attach(9);
  leftProp.attach(10);
  
  rightProp.write(10);
  leftProp.write(10);
  delay(7000);
}

void pid(double &Input,double &Output,int Setpoint,double Kp,double Ki,double Kd )
{ 
     
  timePrev = time; 
  time = millis(); 
  elapsedTime = (time - timePrev) / 1000;
  lasterror = error;
  error = Setpoint - Input;
  totalerror += error;
  deltaerror = error - lasterror;
  Output = Kp*error + (Ki*elapsedTime)*totalerror + (Kd/elapsedTime)*deltaerror;
  if (Output >= 800) Output = 800;
  SignalRight = currentSignalRight + Output;
  SignalLeft = currentSignalLeft - Output;
  if (SignalRight > max_control) SignalRight = max_control;
  if (SignalLeft > max_control) SignalLeft = max_control;
  if (SignalRight < min_control) SignalRight = min_control;
  if (SignalLeft < min_control) SignalLeft = min_control;

  
}

void motorPwm()
{
  if (SignalRight>currentSignalRight) currentSignalRight++;
  if (SignalLeft>currentSignalLeft) currentSignalLeft++;
  if (SignalLeft<currentSignalLeft) currentSignalLeft--;
  if (SignalRight<currentSignalRight) currentSignalRight--;
  
    
}
void loop() {
  gyro.update();
  Input = gyro.getAngleY();
  Setpoint = 0;
  pid(Input,Output,Setpoint,Kp,Ki,Kd);
  motorPwm();
  rightProp.writeMicroseconds(currentSignalRight);
  leftProp.writeMicroseconds(currentSignalLeft);
  delay(10);

}
