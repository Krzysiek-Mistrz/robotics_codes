#include <Servo.h>

#define MOTOR2 11   //DR motor
#define MOTOR3 10   //DL motor
#define MOTOR4 9    //UL motor
#define MOTOR1 3    //UR motor
//servo control
#define MaxMicroseconds 2000
#define MinMicroseconds 1000

Servo upLeftMotor;
int MotorPWM = 1000;  //*MAY NEED CALIB*

void setup() {
  Serial.begin(9600);
  upLeftMotor.attach(MOTOR4, MinMicroseconds, MaxMicroseconds);
}

void loop() {
  Serial.println(MotorPWM);
  upLeftMotor.writeMicroseconds(MotorPWM);
  for (int i = MotorPWM; i < 1600; i++){
    upLeftMotor.writeMicroseconds(i);
    Serial.println(i);
    delay(10);
  }
  delay(10000);
  for (int i = 1600; i > 1000; i--){
    upLeftMotor.writeMicroseconds(i);
    Serial.println(i);
    delay(10);
  }
  delay(10000);
}