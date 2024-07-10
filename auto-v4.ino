#include <ESP32Servo.h>

#include <Ps3Controller.h>

bool nitroState = false;


// Pin para el LED
const uint8_t ledPin = 2; // Cambia al pin que estés utilizando
const uint8_t ledPin2 = 4; // Cambia al pin que estés utilizando

//Right motor
int rightMotorPin2=26;
int rightMotorPin1=27;
//Left motor
int leftMotorPin2=12;
int leftMotorPin1=14;

//Servo
static const int servoPin = 18;
Servo servo1;

int LeftAngle = 90;
int RightAngle = 0;
int Normalpos = 45;

const int MAX_MOTOR_SPEED = 1;


const int NITRO_SPEED = 255;
const int NORMAL_SPEED = 200;

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
  else if(Ps3.data.button.l1)   //Left emote
  {
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if(Ps3.data.button.r1)   //Right emote
  {
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
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
  else{
    servo1.write(Normalpos);
  }

  // nitro
  if (Ps3.event.button_down.cross) {
    nitroState = true;
    ledcWrite(2, NITRO_SPEED);
  }
  else if(Ps3.event.button_up.cross) {
    nitroState = false;
    ledcWrite(2, NORMAL_SPEED);
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
  ledcSetup(2, 12000, 8); // Configura el canal 0, frecuencia de 12 kHz, resolución de 8 bits
  ledcAttachPin(ledPin, 2); // Asigna el pin al canal 0 PWM
  ledcAttachPin(ledPin2, 2); // Asigna el pin al canal 0 PWM
  ledcWrite(2, NORMAL_SPEED);

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