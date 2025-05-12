#include <Servo.h>

Servo myservo;
int pozycja = 0;
int zmiana = 6;

void setup() {
  myservo.attach(3);
}

void loop() {
  if (pozycja < 180) {
    myservo.write(pozycja);
  } else {
    pozycja = 0;
  }    
  pozycja = pozycja + zmiana;
  delay(200);
}