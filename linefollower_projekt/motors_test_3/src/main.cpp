#include <Arduino.h>

// Definicje pinów ESP32-S3 podłączonych do TB6612FNG
const int PWMA = 4;
const int AIN1 = 6;
const int AIN2 = 7;
const int BIN1 = 15;
const int BIN2 = 16;
const int PWMB = 5;

void setup() {
  // Konfiguracja pinów jako wyjścia
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Uruchomienie komunikacji szeregowej do debugowania
  //Serial.begin(115200);
}

void loop() {
  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, HIGH);
  // analogWrite(PWMA, 100);
  // delay(3000);

  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, LOW);
  // analogWrite(PWMA, 0);
  // delay(3000);

  // digitalWrite(AIN1, HIGH);
  // digitalWrite(AIN2, LOW);
  // analogWrite(PWMA, 100);
  // delay(3000);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 100);
  delay(3000);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
  delay(3000);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 100);
  delay(3000);
}
