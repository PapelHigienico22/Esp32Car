#include <Ps3Controller.h>

#include <ESP32_Servo.h>

//Right motor
int rightMotorPin2=14;
int rightMotorPin1=27;
//Left motor
int leftMotorPin2=26;
int leftMotorPin1=12;

//Servo
static const int servoPin = 18;
Servo servo1;

int LeftAngle = 180;
int RightAngle = 0;
int Normalpos = 90;

#define MAX_MOTOR_SPEED 1

void notify()
{
  int xAxisValue =(Ps3.data.analog.stick.lx);  //Left stick  - x axis - right/left car movement

  if (Ps3.data.button.r2)       //Move car Forward
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.l2)   //Move car Backward
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else                           //Stop the car
  {
    rotateMotor(0, 0);

  }

  if(xAxisValue < -20){
    servo1.write(LeftAngle);
  }
  else if(xAxisValue > 20){
    servo1.write(RightAngle);
  }
  else if(xAxisValue < 20 && xAxisValue > -20){
    servo1.write(Normalpos);
  }
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  rotateMotor(0, 0);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  
}

void setUpPinModes()
{
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);

  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);
  
  rotateMotor(0, 0);
}


void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  servo1.attach(servoPin);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin();
  Serial.println("Ready.");
}

void loop()
{
}