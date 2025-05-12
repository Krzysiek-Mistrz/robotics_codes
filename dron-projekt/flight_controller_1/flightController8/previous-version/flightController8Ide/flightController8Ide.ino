#include <Wire.h>
#define PI 3.14159265358979323846


//time vars
float time;
float timePrev;
float elapsedTime;
//GY521 vars
int rateCalib = 0;    //security var for calib
float rateX, rateY, rateZ;  //rate roll, pitch, yaw
float angleXRate, angleYRate;   //angles from gyro
float rateXError, rateYError; //used for calibration
int accCalib = 0; //security var for calib
float radToDeg = 180 / PI;
float accX, accY, accZ;
float angleXAcc, angleYAcc; //angles from gyro
float accAngleXError, accAngleYError; //used for calibration
//complementary filter
float angleX, angleY;


void setup() {
  Wire.begin(); //start wire comunication
  Wire.beginTransmission(0x68); //start communication with gy521          
  Wire.write(0x6B); //reset current config
  Wire.write(0x00);
  Wire.endTransmission(true);   //end transmission
  Wire.beginTransmission(0x68); //start communication for gyro config
  Wire.write(0x1B); //gyro register
  Wire.write(0x10); //settin gyro register (1000dps scale)
  Wire.endTransmission(true);
  Wire.beginTransmission(0x68); //start communication for acc config
  Wire.write(0x1C); //acc register
  Wire.write(0x10); //settin acc register (8g scale)
  Wire.endTransmission(true); 

  Serial.begin(9600);
  time = millis();

  if(accCalib == 0)
  {
    for(int a = 0; a < 2000; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); //acc register address
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);  //16b for 1 info about acc
      accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
      accX = (Wire.read() << 8 | Wire.read()) / 4096.0; //1 reister address = 8b = 2B
      accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
      accAngleXError = accAngleXError + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * radToDeg));  //calculatin sum of acc angles
      accAngleYError = accAngleYError + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * radToDeg));
    }
    accAngleXError = accAngleXError / 2000;
    accAngleYError = accAngleYError / 2000;
    accCalib = 1;
  }
  
  if(rateCalib == 0)
  {
    for(int i = 0; i < 2000; i++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x43); //gyro registr address
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 4, true);  //16b for 1 info about gyro, total of 4 * 8 = 32b here
      rateY = Wire.read() << 8 | Wire.read();
      rateX = Wire.read() << 8 | Wire.read();
      rateXError = rateXError + (rateX / 32.8);   //calculatin sum of rates
      rateYError = rateYError + (rateY / 32.8);
    }
    rateXError = rateXError / 2000;
    rateYError = rateYError / 2000;
    rateCalib = 1;
  }
}


void loop() {
    timePrev = time;
    time = millis();
    elapsedTime = (time - timePrev) / 1000;   //ms -> s : 1 / 1000

    Wire.beginTransmission(0x68);   //startin communication for gyro
    Wire.write(0x43);   //gyro register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    rateY = Wire.read() << 8 | Wire.read();
    rateX = Wire.read() << 8 | Wire.read();
    rateX = (rateX / 32.8) - rateXError;    //32.8 corresponding 1000dps value
    rateY = (rateY / 32.8) - rateYError;  //convertin to deg/s
    angleXRate = rateX * elapsedTime;   //takin discrete for of integral for angle
    angleYRate = rateY * elapsedTime;

    Wire.beginTransmission(0x68);   //startin communication for acc
    Wire.write(0x3B);   //acc register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    accX = (Wire.read() << 8 | Wire.read()) / 4096.0;  //converting to m/s^2
    accZ = (Wire.read() << 8 | Wire.read()) / 4096.0; 
    angleXAcc = (atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * radToDeg) - accAngleXError;
    angleYAcc = (atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * radToDeg) - accAngleYError;    

    angleX = 0.98 * (angleX + angleXRate) + 0.02 * angleXAcc;  //calculatin inclination angles usin complementary filter
    angleY = 0.98 * (angleY + angleYRate) + 0.02 * angleYAcc;
   
    Serial.println("angle x: " + String(angleX) + "\t" + "angle y: " + String(angleY));
}