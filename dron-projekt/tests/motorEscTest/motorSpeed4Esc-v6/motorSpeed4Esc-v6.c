#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>

#define ESCPIN11 11
#define ESCPIN10 10
#define ESCPIN9 9
#define ESCPIN3 3
#define MaxMicroseconds 2000
#define MinMicroseconds 1000

//motor control
Servo ESC11, ESC10, ESC9, ESC3;
int MotorPWM = 1500;
unsigned long Time;
bool wasThereFlight = 0;
//float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void setup() {
  Serial.begin(9600);
  //esc init
  ESC11.attach(ESCPIN11, MinMicroseconds, MaxMicroseconds);
  ESC10.attach(ESCPIN10, MinMicroseconds, MaxMicroseconds);
  ESC9.attach(ESCPIN9, MinMicroseconds, MaxMicroseconds);
  ESC3.attach(ESCPIN3, MinMicroseconds, MaxMicroseconds);
}

void loop() {
    if (wasThereFlight == 0){ //if it wasnt fyling yet
        Time = millis();
        Serial.println(MotorPWM);
        //initializing all 4 esc with min pwm
        ESC11.writeMicroseconds(MotorPWM);
        ESC10.writeMicroseconds(MotorPWM);
        ESC9.writeMicroseconds(MotorPWM);
        ESC3.writeMicroseconds(MotorPWM);
        //waitin for the motor initial setup
        delay(250);
        //increasing throttle
        for (int i = MotorPWM; i < 1800; i++){
        //update();
        //Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput4));
        ESC11.writeMicroseconds(i);// - MotorInput1);
        ESC3.writeMicroseconds(i);// - MotorInput2);
        ESC9.writeMicroseconds(i);// + MotorInput3);
        ESC10.writeMicroseconds(i);// + MotorInput4);
        delay(10);
        }
        Serial.println();
        MotorPWM = 1800;
        //fly up until less than 10s
        //pin<->motor: 11=UPPER_RIGHT=motor1, 10=UPPER_LEFT=motor4, 9=DOWN_LEFT=motor3, 3=DOWN_RIGHT=motor2
        Time = 0;
        while (Time < 10000UL) {
        //update();
        ESC11.writeMicroseconds(MotorPWM);// - MotorInput1);
        ESC3.writeMicroseconds(MotorPWM);// - MotorInput2);
        ESC9.writeMicroseconds(MotorPWM);// + MotorInput3);
        ESC10.writeMicroseconds(MotorPWM);// + MotorInput4);
        Time = millis();
        }
        //decreasing throttle
        for (int i = MotorPWM; i > 1500; i--){
        //update();
        //Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput4));
        ESC11.writeMicroseconds(i);// - MotorInput1);
        ESC3.writeMicroseconds(i);// - MotorInput2);
        ESC9.writeMicroseconds(i);// + MotorInput3);
        ESC10.writeMicroseconds(i);// + MotorInput4);
        delay(50);
        }
    }
}
