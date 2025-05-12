#include <QTRSensors.h>
#include <Arduino.h>

// Definicje czujników
#define NUM_SENSORS 8
#define SENSOR_PINS {14, 13, 12, 11, 10, 9, 46, 3}

const uint8_t sensorPins[NUM_SENSORS] = SENSOR_PINS;
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Definicje silników
const int PWMA = 4;
const int AIN1 = 6;
const int AIN2 = 7;
const int BIN1 = 15;
const int BIN2 = 16;
const int PWMB = 5;

// Parametry prędkości
const uint8_t maxSpeed = 100;
const uint8_t baseSpeed = 40;

// Funkcje przynależności dla zakresu błędu [-3500, 3500]
float membership_NL(int error) {
  if (error <= -3500) return 1.0;
  if (error > -3500 && error < -1750) return (-1750 - error) / 1750.0;
  return 0.0;
}

float membership_NS(int error) {
  if (error <= -3500 || error >= 0) return 0.0;
  if (error > -3500 && error < -1750) return (error + 3500) / 1750.0;
  if (error >= -1750 && error < 0) return (-error) / 1750.0;
  return 0.0;
}

float membership_ZE(int error) {
  if (error <= -1750 || error >= 1750) return 0.0;
  if (error > -1750 && error < 0) return (error + 1750) / 1750.0;
  if (error >= 0 && error < 1750) return (1750 - error) / 1750.0;
  return 0.0;
}

float membership_PS(int error) {
  if (error <= 0 || error >= 3500) return 0.0;
  if (error > 0 && error < 1750) return error / 1750.0;
  if (error >= 1750 && error < 3500) return (3500 - error) / 1750.0;
  return 0.0;
}

float membership_PL(int error) {
  if (error >= 3500) return 1.0;
  if (error > 1750 && error < 3500) return (error - 1750) / 1750.0;
  return 0.0;
}

// Prototyp funkcji kontrolera fuzzy logic
float fuzzyController(int error);

void setup() {
  // Konfiguracja pinów silników jako wyjścia
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Inicjalizacja czujników
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  // Kalibracja czujników
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
}

void loop() {
  // Odczyt pozycji linii
  uint16_t position = qtr.readLineBlack(sensorValues);
  // Obliczenie błędu – idealna pozycja to 3500
  int error = 3500 - position;

  // Obliczenie korekty za pomocą kontrolera fuzzy logic
  float output = fuzzyController(error);

  // Obliczenie prędkości silników z większym zakresem korekty
  int leftMotorSpeed = baseSpeed + (int)output;
  int rightMotorSpeed = baseSpeed - (int)output;

  // Zapewnienie, że prędkości mieszczą się w przedziale [0, maxSpeed]
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // Sterowanie lewym silnikiem
  if (leftMotorSpeed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, leftMotorSpeed);

  // Sterowanie prawym silnikiem
  if (rightMotorSpeed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, rightMotorSpeed);

  delay(10); // Krótszy delay dla lepszej reakcji
}

// Funkcja kontrolera fuzzy logic
float fuzzyController(int error) {
  // Fuzzification – obliczenie stopni przynależności
  float nl = membership_NL(error);
  float ns = membership_NS(error);
  float ze = membership_ZE(error);
  float ps = membership_PS(error);
  float pl = membership_PL(error);

  // Nowe wartości wyjściowe – zwiększony zakres korekty
  float output_nl = -60;
  float output_ns = -30;
  float output_ze = 0;
  float output_ps = 30;
  float output_pl = 60;

  // Agregacja – metoda środka ciężkości
  float numerator = nl * output_nl + ns * output_ns + ze * output_ze + ps * output_ps + pl * output_pl;
  float denominator = nl + ns + ze + ps + pl;

  if (denominator == 0) return 0;
  return numerator / denominator;
}