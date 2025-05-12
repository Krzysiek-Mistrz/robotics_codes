#include <QTRSensors.h>
#include <Arduino.h>

// Definicje czujników
#define NUM_SENSORS 8
#define SENSOR_PINS {14, 13, 12, 11, 10, 9, 46, 3}

const uint8_t sensorPins[NUM_SENSORS] = SENSOR_PINS;
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

const int PWMA = 4;
const int AIN1 = 6;
const int AIN2 = 7;
const int BIN1 = 15;
const int BIN2 = 16;
const int PWMB = 5;

// stale
float Kp = 0.1;
float Ki = 0;
float Kd = 0;
int lastError = 0;
int integral = 0;
const uint8_t maxSpeed = 100;
const uint8_t baseSpeed = 40;

void setup() {
  // Konfiguracja pinów jako wyjścia
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // debug
  //Serial.begin(115200);

  // Inicjalizacja czujników
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  // Kalibracja czujników
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = 3500 - position;

  // PID control
  int P = error;
  integral += error;
  int D = error - lastError;
  int output = Kp * P + Ki * integral + Kd * D;
  lastError = error;

  // Obliczanie prędkości silników
  int leftMotorSpeed = baseSpeed + output;
  int rightMotorSpeed = baseSpeed - output;

  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  // Sterowanie silnikiem lewym
  if (leftMotorSpeed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    leftMotorSpeed = 0;//-leftMotorSpeed;
  }
  analogWrite(PWMA, leftMotorSpeed);

  // Sterowanie silnikiem prawym
  if (rightMotorSpeed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); 
    rightMotorSpeed = 0;//-rightMotorSpeed;
  }
  analogWrite(PWMB, rightMotorSpeed);

  // debug
  //Serial.print("Position: ");
  //Serial.print(position);
  //Serial.print(" Error: ");
  //Serial.print(error);
  //Serial.print(" Left Motor: ");
  //Serial.print(leftMotorSpeed);
  //Serial.print(" Right Motor: ");
  //Serial.println(rightMotorSpeed);

  delay(50);
}
