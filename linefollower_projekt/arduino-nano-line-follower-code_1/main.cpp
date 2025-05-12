#include <Arduino.h>
#define INP1 4
#define INP2 5
#define INP3 6
#define INP4 7
#define ENA1 3
#define ENA2 9
#define ADC0 A0
#define ADC1 A1

int m1Speed = 100;
int m2Speed = 100;
int leftRotationSpeed = 250;
int rightRotationSpeed = 250;

void setup() {
  pinMode(INP1, OUTPUT);
  pinMode(INP2, OUTPUT);
  pinMode(INP3, OUTPUT);
  pinMode(INP4, OUTPUT);
  pinMode(ENA1, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(ADC1, INPUT);
  pinMode(ADC0, INPUT);
}

void loop() {
  int leftSensor = digitalRead(ADC0);
  int rightSensor = digitalRead(ADC1);
  if(rightSensor == 0 && leftSensor == 0) { //jezeli wykrywa na obu -> prosto
    forward();
  }
  else if(rightSensor == 0 && leftSensor == 1) {  //jezeli wykrywa na prawym na lewym nie -> prawo
    right();
  }
  else if(rightSensor == 1 && leftSensor == 0) {  //jezeli wykrywa na lewym prawym nie -> lewo
    left();
  }
  else if(rightSensor == 1 && leftSensor == 1) {  //jezeli nie wykrywa -> zatrzymaj
    stop();
  }
}

void forward()  //m1, m2 -> lewo, lewo
{
  digitalWrite(INP1, HIGH);
  digitalWrite(INP2, LOW);
  digitalWrite(INP3, HIGH);
  digitalWrite(INP4, LOW);
  analogWrite(ENA1, m1Speed);
  analogWrite(ENA2, m2Speed);
}

void backward() //m1, m2 -> prawo, prawo
{
  digitalWrite(INP1, LOW);
  digitalWrite(INP2, HIGH);
  digitalWrite(INP3, LOW);
  digitalWrite(INP4, HIGH);
  analogWrite(ENA1, m1Speed);
  analogWrite(ENA2, m2Speed);
}

void right()  //m1, m2 -> prawo, lewo
{
  digitalWrite(INP1, LOW);
  digitalWrite(INP2, HIGH);
  digitalWrite(INP3, HIGH);
  digitalWrite(INP4, LOW);
  analogWrite(ENA1, leftRotationSpeed);
  analogWrite(ENA2, rightRotationSpeed);
}

void left() //m1, m2 -> lewo, prawo
{
  digitalWrite(INP1, HIGH);
  digitalWrite(INP2, LOW);
  digitalWrite(INP3, LOW);
  digitalWrite(INP4, HIGH);
  analogWrite(ENA1, leftRotationSpeed);
  analogWrite(ENA2, rightRotationSpeed);
}

void stop() //m1, m2 -> brak, brak
{
  digitalWrite(INP1, LOW);
  digitalWrite(INP2, LOW);
  digitalWrite(INP3, LOW);
  digitalWrite(INP4, LOW);
}