#include <Servo.h>
#include <Arduino.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 11
int DELAY = 1800;

Servo motor;
int spd = 2000;
int desiredValue;
int rememberedValue;

void setup() {
  Serial.begin(9600);
  Serial.println("ESC CALIBRATION...");
  Serial.println(" ");
  delay(1500);
  Serial.println("...CALIBRATION...");
  delay(1000);
  motor.attach(MOTOR_PIN);
  Serial.print("WRITING MAXIMUM OUTPUT: (");
  Serial.print(MAX_SIGNAL);
  Serial.print("\n");
  motor.writeMicroseconds(MAX_SIGNAL);
  delay(1000);  // Wait for the ESC to recognize the maximum signal
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("WRITING MINIMUM OUTPUT: (");
  Serial.print(MIN_SIGNAL);
  Serial.print("\n");
  motor.writeMicroseconds(MIN_SIGNAL);
  delay(1000);  // Wait for the ESC to recognize the minimum signal
  Serial.println("===ESC CALIBRATION SUCCESS===");
  for (int pos = MIN_SIGNAL; pos <= MAX_SIGNAL; pos += 1) {
    motor.writeMicroseconds(pos);
    delay(30);
  }
  motor.writeMicroseconds(spd);
}

void loop() {
  if (Serial.available()) {
    desiredValue = Serial.parseInt();
    rememberedValue = desiredValue;
    Serial.println("Desired value: " + String(rememberedValue));
  }

  if (rememberedValue != spd) {
    if (rememberedValue > spd) {
      for (int i = spd; i < rememberedValue; i++) {
        motor.writeMicroseconds(i);
        delay(30);
      }
    } else if (rememberedValue < spd) {
      for (int i = spd; i > rememberedValue; i--) {
        motor.writeMicroseconds(i);
        delay(30);
      }
    }
    spd = rememberedValue;
    Serial.println("Current speed value: " + String(spd));
  }
}
