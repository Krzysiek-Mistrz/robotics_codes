#include <Servo.h>

#define ESCPIN 11
#define POTENPIN A5
#define MAX 2000
#define MIN 1000
Servo ESC;
int potValue;

void setup() {
  Serial.begin(9600);
  ESC.attach(ESCPIN, MIN, MAX);
}

void loop() {
  potValue = analogRead(POTENPIN);
  80 max duszenie; 2 - min duszenie
  //max duszenie dla 90, 
  potValue = map(potValue, 0, 1023, 0, 180);
  Serial.println(potValue);
  ESC.write(potValue);
}