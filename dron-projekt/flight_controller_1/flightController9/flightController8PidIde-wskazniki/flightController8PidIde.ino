#include <Wire.h>
#include <Servo.h>
#define PI 3.14159265358979323846
#define MOTORDR 11   //Down Right motor
#define MOTORDL 10   //Down Left motor
#define MOTORUL 9    //Upper Left motor
#define MOTORUR 3    //Upper Right motor
#define MAXMICROSECONDS 2000
#define MINMICROSECONDS 1000


//motor vars
Servo upperLeftMotor;
Servo downLeftMotor;
Servo upperRightMotor;
Servo downRightMotor;
const int throttleCutOff = 1500;
const int maxFlyable = 1800;
const int minFlyable = 1650;
const float gainConst = 1.0;
bool wasThereFlight = 0;
//pid vars
const double rollPConstant = 0.7; //  1.3
const double rollIConstant = 0.006;   //  0.04
const double rollDConstant = 1.2; //  18
const float desiredXAngle = 0;    //desired roll angle 0 to stabilise
const double pitchPConstant = 0.72;   //  1.3
const double pitchIConstant = 0.006;  //  0.04
const double pitchDConstant = 1.22;   //  18
const float desiredYAngle = 0;    //desired pitch angle
struct PidStruct {
    float rollPid, rollError, rollPrevError;    //vars for roll pid
    float rollPTerm = 0;    //initial values
    float rollITerm = 0;
    float rollDTerm = 0;
    float pitchPid, pitchError, pitchPrevError; //vars for pitch pid
    float pitchPTerm = 0;   //initial values
    float pitchITerm = 0;
    float pitchDTerm = 0;
};
//time vars
unsigned long time;
unsigned long timePrev;
unsigned long elapsedTime;
unsigned long flyTime;
//GY521 vars
const float radToDeg = 180 / PI;
float rateXError, rateYError; //used for calibration
float accAngleXError, accAngleYError; //used for calibration
struct MpuStruct {
    float rateX, rateY, rateZ;  //rate roll, pitch, yaw
    float accX, accY, accZ;
    float angleXRate, angleYRate;  //angles from gyro
    float angleXAcc, angleYAcc;    //angles from gyro
    float angleX, angleY;          //complementary filter
};


void resetPid(PidStruct * pidVars) {   //used for resettin pid after completed fly
    pidVars->rollPrevError = 0; pidVars->pitchPrevError = 0;
    pidVars->rollITerm = 0; pidVars->rollDTerm = 0; pidVars->rollPTerm = 0;
    pidVars->pitchITerm = 0; pidVars->pitchDTerm = 0; pidVars->pitchPTerm = 0;
    pidVars->rollPid = 0; pidVars->pitchPid = 0;
}


void resetMotors(void) {  //used to reset motors after flight
    upperLeftMotor.writeMicroseconds(throttleCutOff);
    upperRightMotor.writeMicroseconds(throttleCutOff);
    downLeftMotor.writeMicroseconds(throttleCutOff);
    downRightMotor.writeMicroseconds(throttleCutOff);
}


void impCalibTst(void) {
    Wire.beginTransmission(0x68); //rozpoczecie komunikacji z adresem 0x68 (MPU6050)
    bool error = Wire.endTransmission();
    if (error == 0) { //modul MPU6050 nie jest podlaczony
        //migotanie wbudowanej diody w nieskonczonosc
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
}


void calibMpu(void) {
    float rateX, rateY, rateZ;  //rate roll, pitch, yaw
    float accX, accY, accZ;
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
    for (int a = 0; a < 2000; a++) {  //calibration for accelerometer
        Wire.beginTransmission(0x68);
        Wire.write(0x3B);  //acc register address
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 6, true);  //16b for 1 info about acc
        accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
        accX = (Wire.read() << 8 | Wire.read()) / 4096.0;  //1 reister address = 8b = 2B
        accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
        accAngleXError = accAngleXError + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * radToDeg));  //calculatin sum of acc angles
        accAngleYError = accAngleYError + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * radToDeg));
    }
    accAngleXError = accAngleXError / 2000;
    accAngleYError = accAngleYError / 2000;
    for (int i = 0; i < 2000; i++) {  //calibration for gyro
        Wire.beginTransmission(0x68);
        Wire.write(0x43);  //gyro registr address
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 4, true);  //16b for 1 info about gyro, total of 4 * 8 = 32b here
        rateY = Wire.read() << 8 | Wire.read();
        rateX = Wire.read() << 8 | Wire.read();
        rateXError = rateXError + (rateX / 32.8);  //calculatin sum of rates
        rateYError = rateYError + (rateY / 32.8);
    }
    rateXError = rateXError / 2000;
    rateYError = rateYError / 2000;
}


void anglesRead(MpuStruct * mpuVars) {
    Wire.beginTransmission(0x68);   //startin communication for gyro
    Wire.write(0x43);   //gyro register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    mpuVars->rateY = Wire.read() << 8 | Wire.read(); //readin y first beacause of mpu placement on drone
    mpuVars->rateX = Wire.read() << 8 | Wire.read();
    mpuVars->rateX = (mpuVars->rateX / 32.8) - rateXError;    //32.8 corresponding 1000dps value
    mpuVars->rateY = (mpuVars->rateY / 32.8) - rateYError;  //convertin to deg/s
    mpuVars->angleXRate = mpuVars->rateX * elapsedTime;   //takin discrete for of integral for angle
    mpuVars->angleYRate = mpuVars->rateY * elapsedTime;
    Wire.beginTransmission(0x68);   //startin communication for acc
    Wire.write(0x3B);   //acc register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    mpuVars->accY = (Wire.read() << 8 | Wire.read()) / 4096.0;   //readin y first beacause of mpu placement on drone
    mpuVars->accX = (Wire.read() << 8 | Wire.read()) / 4096.0;  //converting to m/s^2
    mpuVars->accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
    mpuVars->angleXAcc = (atan((mpuVars->accY) / sqrt(pow((mpuVars->accX), 2) + pow((mpuVars->accZ), 2))) * radToDeg) - accAngleXError;
    mpuVars->angleYAcc = (atan(-1 * (mpuVars->accX) / sqrt(pow((mpuVars->accY), 2) + pow((mpuVars->accZ), 2))) * radToDeg) - accAngleYError;    
    mpuVars->angleX = 0.9996 * (mpuVars->angleX + mpuVars->angleXRate) + 0.0004 * mpuVars->angleXAcc;  //calculatin inclination angles usin complementary filter
    mpuVars->angleY = 0.9996 * (mpuVars->angleY + mpuVars->angleYRate) + 0.0004 * mpuVars->angleYAcc;
}


void pidEquation(PidStruct * pidVars, MpuStruct * mpuVars) {
    pidVars->rollError = mpuVars->angleX - desiredXAngle;  //difference between current angle and desired one
    pidVars->pitchError = mpuVars->angleY - desiredYAngle;
    pidVars->rollPTerm = rollPConstant * pidVars->rollError; //proportional terms of roll and pitch
    pidVars->pitchPTerm = pitchPConstant * pidVars->pitchError;
    pidVars->rollITerm = pidVars->rollITerm + (rollIConstant * (pidVars->rollError + pidVars->rollPrevError) * (elapsedTime / 2));    //integral term to smooth little error bet. -3deg:3deg
    pidVars->pitchITerm = pidVars->pitchITerm + (pitchIConstant * (pidVars->pitchError + pidVars->pitchPrevError) * (elapsedTime / 2));
    if(pidVars->rollITerm > 400)   pidVars->rollITerm = 400;
    else if(pidVars->rollITerm < -400)   pidVars->rollITerm = -400;
    if(pidVars->pitchITerm > 400)   pidVars->pitchITerm = 400;
    else if(pidVars->pitchITerm < -400)   pidVars->pitchITerm = -400;
    pidVars->rollDTerm = rollDConstant * ((pidVars->rollError - pidVars->rollPrevError) / elapsedTime);   //derivative term = DConst*(dE/dt)
    pidVars->pitchDTerm = pitchDConstant * ((pidVars->pitchError - pidVars->pitchPrevError) / elapsedTime);
    pidVars->rollPid = pidVars->rollPTerm + pidVars->rollITerm + pidVars->rollDTerm;   //final "calib" values from pid
    pidVars->pitchPid = pidVars->pitchPTerm + pidVars->pitchITerm + pidVars->pitchDTerm;
    if(pidVars->rollPid < -400) {
        pidVars->rollPid = -400;   //security check to not exceed min/max pwm
    }
    if(pidVars->rollPid > 400) {
        pidVars->rollPid = 400;
    }
    if(pidVars->pitchPid < -400) {
        pidVars->pitchPid = -400;
    }
    if(pidVars->pitchPid > 400) {
        pidVars->pitchPid = 400;
    }
}


void update(int throttleValue, PidStruct * pidVars) {
    float upperLeftPwm, downLeftPwm, upperRightPwm, downRightPwm;
    float upperLeftPwmFinal, downLeftPwmFinal, upperRightPwmFinal, downRightPwmFinal;
    timePrev = time;
    time = millis();    //one loop iteration time used for integration
    elapsedTime = (time - timePrev) / 1000;   //ms -> s : 1 / 1000
    upperRightPwm = gainConst * (-(pidVars->rollPid) - pidVars->pitchPid);   //motors pwm calculation
    downRightPwm = gainConst * (-(pidVars->rollPid) + pidVars->pitchPid);
    downLeftPwm = gainConst * (pidVars->rollPid + pidVars->pitchPid);
    upperLeftPwm = gainConst * (pidVars->rollPid - pidVars->pitchPid);
    upperLeftPwmFinal = upperLeftPwm + throttleValue;
    downLeftPwmFinal = downLeftPwm + throttleValue;
    upperRightPwmFinal = upperRightPwm + throttleValue;
    downRightPwmFinal = downRightPwm + throttleValue;
    if(upperRightPwmFinal < 1100) {  //security check to not exceed min/max pwm
        upperRightPwmFinal = 1100;
    }
    if(upperRightPwmFinal > 2000) {
        upperRightPwmFinal = 2000;
    }
    if(upperLeftPwmFinal < 1100) {
        upperLeftPwmFinal = 1100;
    }
    if(upperLeftPwmFinal > 2000) {
        upperLeftPwmFinal = 2000;
    }
    if(downRightPwmFinal < 1100)
    {
        downRightPwmFinal = 1100;
    }
    if(downRightPwmFinal > 2000) {
        downRightPwmFinal = 2000;
    }
    if(downLeftPwmFinal < 1100) {
        downLeftPwmFinal = 1100;
    }
    if(downLeftPwmFinal > 2000) {
        downLeftPwmFinal = 2000;
    }
    pidVars->rollPrevError = pidVars->rollError; //storin prev errors for roll and pitch
    pidVars->pitchPrevError = pidVars->pitchError;
    upperLeftMotor.writeMicroseconds(upperLeftPwmFinal); 
    downLeftMotor.writeMicroseconds(downLeftPwmFinal);
    upperRightMotor.writeMicroseconds(upperRightPwmFinal); 
    downRightMotor.writeMicroseconds(downRightPwmFinal);
    Serial.println("UR, DR: " + String(upperRightPwmFinal) + " " + String(downRightPwmFinal) + "UL, DL: " + String(upperLeftPwmFinal) + " " + String(downLeftPwmFinal));
}


void fly(PidStruct * pidVars, MpuStruct * mpuVars) {
    if (wasThereFlight == 0) {
        for (int i = throttleCutOff; i < maxFlyable; i++) { //increasing throttle from 1500
            update(i, pidVars);
        }
        flyTime = millis();
        while (millis() - flyTime < 5000UL) { //fly up until less than 5s
            anglesRead(mpuVars); //readin angles
            pidEquation(pidVars, mpuVars);  //readin final pid for roll and pitch angles
            update(maxFlyable, pidVars);
        }
        pidVars->rollPid = 0; pidVars->pitchPid = 0;
        for (int i = maxFlyable; i > minFlyable; i--) { //decreasing throttle
            update(i, pidVars);
        }
        pidVars->rollPid = 0; pidVars->pitchPid = 0;
        flyTime = millis();
        while (millis() - flyTime < 20000UL) { //falling
            anglesRead(mpuVars); //readin angles
            pidEquation(pidVars, mpuVars);
            update(minFlyable, pidVars);
        }
        pidVars->rollPid = 0; pidVars->pitchPid = 0;
        for (int i = minFlyable; i > throttleCutOff; i--) { //slowin motors to no rotation pwm value
            update(i, pidVars);
        }
        wasThereFlight = 1; //security check
        resetPid(pidVars);
        resetMotors();
    }
}


void setup() {
    pinMode(LED_BUILTIN, HIGH);
    impCalibTst();
    upperLeftMotor.attach(MOTORUL, MINMICROSECONDS, MAXMICROSECONDS); //left front motor
    downLeftMotor.attach(MOTORDL, MINMICROSECONDS, MAXMICROSECONDS); //left back motor
    upperRightMotor.attach(MOTORUR, MINMICROSECONDS, MAXMICROSECONDS); //right front motor
    downRightMotor.attach(MOTORDR, MINMICROSECONDS, MAXMICROSECONDS); //right back motor
    delay(250); //for esc conf
    calibMpu();    //gy521 calibration
    time = millis();
    delay(250); //for 1st dt
    Serial.begin(9600);
}


void loop() {
    MpuStruct mpu;
    PidStruct pid;
    fly(&pid, &mpu);
}