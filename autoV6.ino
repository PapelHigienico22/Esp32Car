// Logica ledcAttach actualizada
#include <ESP32Servo.h>
#include <Ps3Controller.h>

// Define LED Pins
#define LED1_PIN 33
#define LED2_PIN 32
#define LED3_PIN 22

// Define Buzzer Pin
#define BUZZER_PIN 23

// Variables to hold LED states
bool ledSequenceActive = false;
int colorStep = 0;

bool buzzer_state = false;

bool nitroState = false;

// Pines para los LEDs
const uint8_t ledPin = 2; // Cambia al pin que estés utilizando
const uint8_t ledPin2 = 4; // Cambia al pin que estés utilizando

// Pines para los motores
int rightMotorPin2 = 27;
int rightMotorPin1 = 26;
int leftMotorPin2 = 17;
int leftMotorPin1 = 14;

// Pin para el servo
static const int servoPin = 25;
Servo servo1;

int LeftAngle = 65;
int RightAngle = 0;
int Normalpos = 25;

const int MAX_MOTOR_SPEED = 255; // Cambiado a 255 para máxima velocidad PWM
const int NITRO_SPEED = 255;
const int NORMAL_SPEED = 200;

// Configuración de los canales PWM
#define LEDC_FREQ 12000 // Frecuencia en Hz
#define LEDC_RESOLUTION 8 // Resolución en bits

// Definición de los canales LEDC
#define LEDC_CHANNEL_1 0
#define LEDC_CHANNEL_2 1

// RGB LED channels
#define LEDC_CHANNEL_R 2
#define LEDC_CHANNEL_G 3
#define LEDC_CHANNEL_B 4

void notify() {
  int xAxisValue = Ps3.data.analog.stick.lx;  // Left stick - x axis - right/left car movement

  if (Ps3.data.button.r2) { // Move car Forward
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.l2) { // Move car Backward
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.l1) { // Left emote
    rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (Ps3.data.button.r1) { // Right emote
    rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else { // Stop the car
    rotateMotor(0, 0);
  }

  if (xAxisValue < -20) {
    servo1.write(LeftAngle);
  }
  else if (xAxisValue > 20) {
    servo1.write(RightAngle);
  }
  else {
    servo1.write(Normalpos);
  }

  // Nitro
  if (Ps3.event.button_down.cross) {
    ledcWrite(LEDC_CHANNEL_1, NITRO_SPEED);
    ledcWrite(LEDC_CHANNEL_2, NITRO_SPEED);
  }
  else if (Ps3.event.button_up.cross) {
    ledcWrite(LEDC_CHANNEL_1, NORMAL_SPEED);
    ledcWrite(LEDC_CHANNEL_2, NORMAL_SPEED);
  }

    // Triangle Button - Toggle LED sequence
  if (Ps3.event.button_down.triangle) {
    Serial.println("Triangle pressed");
    ledSequenceActive = !ledSequenceActive; // Toggle sequence state
    if (!ledSequenceActive) {
      // Turn off RGB LED
      ledcWrite(LED1_PIN, 0);
      ledcWrite(LED2_PIN, 0);
      ledcWrite(LED3_PIN, 0);
    }
  }

  // Square Button - Control Buzzer
  if (Ps3.event.button_down.square) {
    Serial.println("Square pressed");
    buzzer_state = true;
    digitalWrite(BUZZER_PIN, buzzer_state); // Turn buzzer on
  }
  if (Ps3.event.button_up.square) {
    Serial.println("Square released");
    buzzer_state = false;
    digitalWrite(BUZZER_PIN, buzzer_state);  // Turn buzzer off
  }
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisconnect() {
  rotateMotor(0, 0);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Control del motor derecho
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // Control del motor izquierdo
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
}

void setUpPinModes() {
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  rotateMotor(0, 0);
}

void setup() {
  // Configurar los canales PWM
  ledcAttachChannel(ledPin, LEDC_FREQ, LEDC_RESOLUTION, LEDC_CHANNEL_1);
  ledcAttachChannel(ledPin2, LEDC_FREQ, LEDC_RESOLUTION, LEDC_CHANNEL_2);
  
  // Configurar el valor inicial de los LEDs
  ledcWrite(LEDC_CHANNEL_1, NORMAL_SPEED);
  ledcWrite(LEDC_CHANNEL_2, NORMAL_SPEED);

  // Set LED pins as outputs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Attach LEDC channels for LEDs
  ledcAttach(LED1_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttach(LED2_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttach(LED3_PIN, LEDC_FREQ, LEDC_RESOLUTION);

  setUpPinModes();
  Serial.begin(115200);
  servo1.attach(servoPin);

  // Inicializar el controlador PS3 con la dirección MAC
  Ps3.begin("c8:f0:9e:51:c6:ec");
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);

  Serial.println("Ready.");
}

void loop() {
  if (!Ps3.isConnected()) return;

  if (ledSequenceActive) {
    // Update RGB LED color
    updateRGBColor();
    delay(100);  // Adjust the delay to control the speed of the color change
  }
}


void updateRGBColor() {
  // Calculate color values based on colorStep
  uint8_t red = (colorStep * 255) / 255;
  uint8_t green = ((255 - colorStep) * 255) / 255;
  uint8_t blue = ((colorStep % 128) * 255) / 128;

  // Set the RGB color
  ledcWrite(LED1_PIN, red);
  ledcWrite(LED2_PIN, green);
  ledcWrite(LED3_PIN, blue);

  // Update color step
  colorStep++;
  if (colorStep > 255) {
    colorStep = 0;
  }
}
