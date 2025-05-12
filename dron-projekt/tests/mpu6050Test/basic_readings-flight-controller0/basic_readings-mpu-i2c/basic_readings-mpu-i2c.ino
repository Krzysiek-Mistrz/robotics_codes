#include <Wire.h>
float RateRoll, RatePitch, RateYaw;

void gyroSignals(void) {
  //from register of gy521 setting comm via i2c
  Wire.beginTransmission(0x68);
  //setting low pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  //setting sensitivity of gyro to 500deg/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  //accessing register holding gyro data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  //readin rad/s from gyro
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  //converting lsb sensitivity to degrees/s
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
}

void setup() {
  Serial.begin(9600);
  //setting clock speed of i2c
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  //starting gyro in power mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  gyroSignals();
  Serial.println("RATE ROLL, RATE PITCH, RATE YAW: " + String(RateRoll) + " " + String(RatePitch) + " " + String(RateYaw));
  Serial.println("-----------------------");
  delay(500);
}