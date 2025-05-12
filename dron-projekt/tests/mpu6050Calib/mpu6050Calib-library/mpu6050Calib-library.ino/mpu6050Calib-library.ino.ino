#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

void gyroSignals() {
  //Get new sensor events with the readings
  mpu.getEvent(&a, &g, &temp);
  //settin sensor data to variables
  RateRoll=(float)g.gyro.y * 50;
  RatePitch=(float)g.gyro.x * 50;
  RateYaw=(float)g.gyro.z * 50;
  //gyro.y switched with gyro.x because of current applications
}

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

  //calibration (for 2 sec)
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyroSignals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

void loop() {
  gyroSignals();
  //substractin from measured current values values from calib
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  Serial.println(String("RATE ROLL, RATE PITCH, RATE YAW: ") + String(RateRoll) + " " + String(RatePitch) + " " + String(RateYaw) + " ");
  Serial.println("==================");
  delay(500);
}