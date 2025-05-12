#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(9600);
  //checkin if mpu connected
  if (!mpu.begin()) {
    Serial.println("ERROR: FAILED TO FIND GY521");
    while (1) {
      delay(10);
    }
  }
  //setting max measured range of acceleration -8G to 8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //mpu.getAccelerometerRange()
  //setting max measured range of acceleration -500deg/s to 500deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //mpu.getGyroRange()
  //settin range of passed frequency - LOW PASS FILTER
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  //mpu.getFilterBandwidth()
  delay(100);
}

void loop() {
  //Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //printin the values of acceleration m/s^2 accelerometer
  //Serial.println("ACC X, ACC Y, ACC Z: " + String(a.acceleration.x) + " " + String(a.acceleration.y) + " " + String(a.acceleration.z));
  //printin the values of gyro rot. x = rate roll, rot. y = rate pitch, rot. z = rate yaw, and converting to deg/s
  Serial.println(String("RATE ROLL, RATE PITCH, RATE YAW: ") + " " + String((float)g.gyro.x * 50) + " " + String((float)g.gyro.y * 50) + " " + String((float)g.gyro.z * 50));
  //printin temp *C
  //Serial.println("Temperature: " + " " + String(temp.temperature));
  Serial.println("-----------------------");
  delay(500);
}