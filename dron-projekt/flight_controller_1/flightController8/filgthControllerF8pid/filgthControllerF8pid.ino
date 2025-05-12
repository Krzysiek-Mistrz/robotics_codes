#include <Wire.h>
#include <Servo.h>


// Constants
const int motorDownRightConstant = 11;   // Down Right motor
const int motorDownLeftConstant = 10;    // Down Left motor
const int motorUpLeftConstant = 9;       // Upper Left motor
const int motorUpRightConstant = 3;      // Upper Right motor
const double piConst = 3.14159265358979323846;
const int maxMicrosecondsConst = 2000;
const int minMicrosecondsConst = 1000;
// Motor variables
struct MotorVars {
    Servo upperLeftMotor;
    Servo upperRightMotor;
    Servo downLeftMotor;
    Servo downRightMotor;
    int inputYaw;
    int inputPitch;
    int inputRoll;
    int throttleCutOff = 1500;
    int maxFlyable = 1660;  // +140 from pid, = 1800
    int minFlyable = 1510;  // +140 from pid, = 1650
    int motorsActivated = 0;
    float upperLeftPwm, downLeftPwm, upperRightPwm, downRightPwm;
    float upperLeftPwmFinal, downLeftPwmFinal, upperRightPwmFinal, downRightPwmFinal;
    float gainConst = 1.2;
    bool wasThereFlight = 0;
};
MotorVars motors;
// PID variables
struct PidVars {
    float rollPid, rollError, rollPrevError;    // Vars for roll PID
    float rollPTerm = 0;    // Initial values
    float rollITerm = 0;
    float rollDTerm = 0;
    double rollPConstant = 0.7; // 3.55
    double rollIConstant = 0.006;   // 0.003
    double rollDConstant = 1.2; // 2.05
    float desiredXAngle = 0;    // Desired roll angle 0 to stabilize
    float pitchPid, pitchError, pitchPrevError; // Vars for pitch PID
    float pitchPTerm = 0;   // Initial values
    float pitchITerm = 0;
    float pitchDTerm = 0;
    double pitchPConstant = 0.72;   // 3.55
    double pitchIConstant = 0.006;  // 0.003
    double pitchDConstant = 1.22;   // 2.05
    float desiredYAngle = 0;    // Desired pitch angle
};
PidVars pid;
// Time variables
struct TimeVars {
    float timeCurr;
    float timePrev;
    float elapsedTime;
    unsigned long flyTime;
};
TimeVars time;
// GY521 variables
struct MpuVars {
    int rateCalib = 0;    // Security var for calibration
    float rateX, rateY, rateZ;  // Rate roll, pitch, yaw
    float angleXRate, angleYRate;   // Angles from gyro
    float rateXError, rateYError; // Used for calibration
    int accCalib = 0; // Security var for calibration
    float radToDeg = 180 / piConst;
    float accX, accY, accZ;
    float angleXAcc, angleYAcc; // Angles from gyro
    float accAngleXError, accAngleYError; // Used for calibration
    float angleX, angleY;   // Complementary filter
};
MpuVars mpu;


void resetPid() {   // Used for resetting PID after completed flight
    pid.rollPrevError = 0;
    pid.pitchPrevError = 0;
    pid.rollITerm = 0;
    pid.rollDTerm = 0;
    pid.rollPTerm = 0;
    pid.pitchITerm = 0;
    pid.pitchDTerm = 0;
    pid.pitchPTerm = 0;
}


void resetMotors() {
    motors.upperLeftMotor.writeMicroseconds(motors.throttleCutOff);
    motors.upperRightMotor.writeMicroseconds(motors.throttleCutOff);
    motors.downLeftMotor.writeMicroseconds(motors.throttleCutOff);
    motors.downRightMotor.writeMicroseconds(motors.throttleCutOff);
}


void impCalibTst() {
    Wire.beginTransmission(0x68); //rozpoczecie komunikacji z adresem 0x68 (MPU6050)
    byte error = Wire.endTransmission();
    if (error != 0) { //modul MPU6050 nie jest podlaczony
        //migotanie wbudowanej diody w nieskonczonosc
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
}


void calibMpu(MpuVars * mpuStruct) {
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
    if(mpuStruct->accCalib == 0) { //calibration for accelerometer
        for(int a = 0; a < 2000; a++) {
            Wire.beginTransmission(0x68);
            Wire.write(0x3B); //acc register address
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 6, true);  //16b for 1 info about acc
            mpuStruct->accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
            mpuStruct->accX = (Wire.read() << 8 | Wire.read()) / 4096.0; //1 reister address = 8b = 2B
            mpuStruct->accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
            (*mpuStruct).accAngleXError = (*mpuStruct).accAngleXError + ((atan((mpuStruct->accY) / sqrt(pow((mpuStruct->accX), 2) + pow((mpuStruct->accZ), 2))) * mpuStruct->radToDeg));  //calculatin sum of acc angles
            (*mpuStruct).accAngleYError = (*mpuStruct).accAngleYError + ((atan(-1 * (mpuStruct->accX) / sqrt(pow((mpuStruct->accY), 2) + pow((mpuStruct->accZ), 2))) * mpuStruct->radToDeg));
        }
        (*mpuStruct).accAngleXError = mpuStruct->accAngleXError / 2000;
        (*mpuStruct).accAngleYError = mpuStruct->accAngleYError / 2000;
        mpuStruct->accCalib = 1;
    }
    if(mpuStruct->rateCalib == 0) {    //calibration for gyro
        for(int i = 0; i < 2000; i++) {
            Wire.beginTransmission(0x68);
            Wire.write(0x43); //gyro registr address
            Wire.endTransmission(false);
            Wire.requestFrom(0x68, 4, true);  //16b for 1 info about gyro, total of 4 * 8 = 32b here
            mpuStruct->rateY = Wire.read() << 8 | Wire.read();
            mpuStruct->rateX = Wire.read() << 8 | Wire.read();
            (*mpuStruct).rateXError = (*mpuStruct).rateXError + (mpuStruct->rateX / 32.8);   //calculatin sum of rates
            (*mpuStruct).rateYError = (*mpuStruct).rateYError + (mpuStruct->rateY / 32.8);
        }
        mpuStruct->rateXError = mpuStruct->rateXError / 2000;
        mpuStruct->rateYError = mpuStruct->rateYError / 2000;
        mpuStruct->rateCalib = 1;
    }
}


void anglesRead(MpuVars * mpuStruct, TimeVars * timeStruct) {
    Wire.beginTransmission(0x68);   //startin communication for gyro
    Wire.write(0x43);   //gyro register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    mpuStruct->rateY = Wire.read() << 8 | Wire.read(); //readin y first beacause of mpu placement on drone
    mpuStruct->rateX = Wire.read() << 8 | Wire.read();
    mpuStruct->rateX = (mpuStruct->rateX / 32.8) - mpuStruct->rateXError;    //32.8 corresponding 1000dps value
    mpuStruct->rateY = (mpuStruct->rateY / 32.8) - mpuStruct->rateYError;  //convertin to deg/s
    mpuStruct->angleXRate = mpuStruct->rateX * timeStruct->elapsedTime;   //takin discrete for of integral for angle
    mpuStruct->angleYRate = mpuStruct->rateY * timeStruct->elapsedTime;
    Wire.beginTransmission(0x68);   //startin communication for acc
    Wire.write(0x3B);   //acc register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    mpuStruct->accY = (Wire.read() << 8 | Wire.read()) / 4096.0;   //readin y first beacause of mpu placement on drone
    mpuStruct->accX = (Wire.read() << 8 | Wire.read()) / 4096.0;  //converting to m/s^2
    mpuStruct->accZ = (Wire.read() << 8 | Wire.read()) / 4096.0; 
    (*mpuStruct).angleXAcc = (atan((mpuStruct->accY) / sqrt(pow((mpuStruct->accX), 2) + pow((mpuStruct->accZ), 2))) * mpuStruct->radToDeg) - mpuStruct->accAngleXError;
    (*mpuStruct).angleYAcc = (atan(-1 * (mpuStruct->accX) / sqrt(pow((mpuStruct->accY), 2) + pow((mpuStruct->accZ), 2))) * mpuStruct->radToDeg) - mpuStruct->accAngleYError;    
    (*mpuStruct).angleX = 0.98 * (mpuStruct->angleX + mpuStruct->angleXRate) + 0.02 * mpuStruct->angleXAcc;  //calculatin inclination angles usin complementary filter
    (*mpuStruct).angleY = 0.98 * (mpuStruct->angleY + mpuStruct->angleYRate) + 0.02 * mpuStruct->angleYAcc;
}


void pidEquation(PidVars * pidStruct, MpuVars * mpuStruct, TimeVars * timeStruct) {
    pidStruct->desiredXAngle = 0;   //pid desired angles of inclination we would normally inputh here values from communication modules 
    pidStruct->desiredYAngle = 0;
    pidStruct->rollError = mpuStruct->angleX - pidStruct->desiredXAngle;  //difference between current angle and desired one
    pidStruct->pitchError = mpuStruct->angleY - pidStruct->desiredYAngle;
    pidStruct->rollPTerm = pidStruct->rollPConstant * pidStruct->rollError; //proportional terms of roll and pitch
    pidStruct->pitchPTerm = pidStruct->pitchPConstant * pidStruct->pitchError;
    pidStruct->rollITerm = pidStruct->rollITerm + (pidStruct->rollIConstant * (pidStruct->rollError + pidStruct->rollPrevError) * (timeStruct->elapsedTime / 2));    //integral term to smooth little error bet. -3deg:3deg
    pidStruct->pitchITerm = pidStruct->pitchITerm + (pidStruct->pitchIConstant * (pidStruct->pitchError + pidStruct->pitchPrevError) * (timeStruct->elapsedTime / 2));
    if(pidStruct->rollITerm > 400)   pidStruct->rollITerm = 400;
    else if(pidStruct->rollITerm < -400)   pidStruct->rollITerm = -400;
    if(pidStruct->pitchITerm > 400)   pidStruct->pitchITerm = 400;
    else if(pidStruct->pitchITerm < -400)   pidStruct->pitchITerm = -400;
    pidStruct->rollDTerm = pidStruct->rollDConstant * ((pidStruct->rollError - pidStruct->rollPrevError) / timeStruct->elapsedTime);   //derivative term = DConst*(dE/dt)
    pidStruct->pitchDTerm = pidStruct->pitchDConstant * ((pidStruct->pitchError - pidStruct->pitchPrevError) / timeStruct->elapsedTime);
    pidStruct->rollPid = pidStruct->rollPTerm + pidStruct->rollITerm + pidStruct->rollDTerm;   //final "calib" values from pid
    pidStruct->pitchPid = pidStruct->pitchPTerm + pidStruct->pitchITerm + pidStruct->pitchDTerm;
    if(pidStruct->rollPid < -400) {
        pidStruct->rollPid = -400;   //security check to not exceed min/max pwm
    }
    if(pidStruct->rollPid > 400) {
        pidStruct->rollPid = 400; 
    }
    if(pidStruct->pitchPid < -400) {
        pidStruct->pitchPid = -400;
    }
    if(pidStruct->pitchPid > 400) {
        pidStruct->pitchPid = 400;
    }
}


void update(int throttleValue, MotorVars * motorsStruct, TimeVars * timeStruct, PidVars * pidStruct) {
    timeStruct->timePrev = timeStruct->timeCurr;
    timeStruct->timeCurr = millis();    //one loop iteration time used for integration
    timeStruct->elapsedTime = (timeStruct->timeCurr - timeStruct->timePrev) / 1000;   //ms -> s : 1 / 1000
    anglesRead(&mpu, &time); //readin angles
    pidEquation(&pid, &mpu, &time);  //readin final pid for roll and pitch angles
    motorsStruct->upperRightPwm = motorsStruct->gainConst * (115 - pidStruct->rollPid - pidStruct->pitchPid);   //motors pwm calculation
    motorsStruct->downRightPwm = motorsStruct->gainConst * (115 - pidStruct->rollPid + pidStruct->pitchPid);
    motorsStruct->downLeftPwm = motorsStruct->gainConst * (115 + pidStruct->rollPid + pidStruct->pitchPid);
    motorsStruct->upperLeftPwm = motorsStruct->gainConst * (115 + pidStruct->rollPid - pidStruct->pitchPid);
    motorsStruct->upperLeftPwmFinal = motorsStruct->upperLeftPwm + throttleValue;
    motorsStruct->downLeftPwmFinal = motorsStruct->downLeftPwm + throttleValue;
    motorsStruct->upperRightPwmFinal = motorsStruct->upperRightPwm + throttleValue;
    motorsStruct->downRightPwmFinal = motorsStruct->downRightPwm + throttleValue;
    if(motorsStruct->upperRightPwmFinal < 1100) {  //security check to not exceed min/max pwm
        motorsStruct->upperRightPwmFinal = 1100;
    }
    if(motorsStruct->upperRightPwmFinal > 2000) {
        motorsStruct->upperRightPwmFinal = 2000;
    }
    if(motorsStruct->upperLeftPwmFinal < 1100) {
        motorsStruct->upperLeftPwmFinal = 1100;
    }
    if(motorsStruct->upperLeftPwmFinal > 2000) {
        motorsStruct->upperLeftPwmFinal = 2000;
    }
    if(motorsStruct->downRightPwmFinal < 1100)
    {
        motorsStruct->downRightPwmFinal = 1100;
    }
    if(motorsStruct->downRightPwmFinal > 2000) {
        motorsStruct->downRightPwmFinal = 2000;
    }
    if(motorsStruct->downLeftPwmFinal < 1100) {
        motorsStruct->downLeftPwmFinal = 1100;
    }
    if(motorsStruct->downLeftPwmFinal > 2000) {
        motorsStruct->downLeftPwmFinal = 2000;
    }
    pidStruct->rollPrevError = pidStruct->rollError; //storin prev errors for roll and pitch
    pidStruct->pitchPrevError = pidStruct->pitchError;
    if(motorsStruct->motorsActivated) { //security check after flight motorsActivated = 0
        motorsStruct->upperLeftMotor.writeMicroseconds(motorsStruct->upperLeftPwmFinal); 
        motorsStruct->downLeftMotor.writeMicroseconds(motorsStruct->downLeftPwmFinal);
        motorsStruct->upperRightMotor.writeMicroseconds(motorsStruct->upperRightPwmFinal); 
        motorsStruct->downRightMotor.writeMicroseconds(motorsStruct->downRightPwmFinal);
    }
    if(!(motorsStruct->motorsActivated))
    {
        motorsStruct->upperLeftMotor.writeMicroseconds(1500); 
        motorsStruct->downLeftMotor.writeMicroseconds(1500);
        motorsStruct->upperRightMotor.writeMicroseconds(1500); 
        motorsStruct->downRightMotor.writeMicroseconds(1500);
    }
    Serial.println("UR, DR: " + String(motorsStruct->upperRightPwmFinal) + " " + String(motorsStruct->downRightPwmFinal) + "UL, DL: " + String(motorsStruct->upperLeftPwmFinal) + " " + String(motorsStruct->downLeftPwmFinal));
}


void fly(MotorVars * motorStruct, TimeVars * timeStruct) {
    if (motorStruct->wasThereFlight == 0) {
        for (int throttle = 1360; throttle < motorStruct->maxFlyable; throttle++) { //increasing throttle from 1500
            update(throttle, motorStruct, timeStruct, &pid);
        }
        timeStruct->flyTime = millis();
        while (millis() - timeStruct->flyTime < 5000UL) { //fly up until less than 5s
            update(motorStruct->maxFlyable, motorStruct, timeStruct, &pid);
        }
        for (int throttle = motorStruct->maxFlyable; throttle > motorStruct->minFlyable; throttle--) { //decreasing throttle
            update(throttle, motorStruct, timeStruct, &pid);
        }
        timeStruct->flyTime = millis();
        while (millis() - timeStruct->flyTime < 20000UL) { //falling
            update(motorStruct->minFlyable, motorStruct, timeStruct, &pid);
        }
        for (int throttle = motorStruct->minFlyable; throttle > 1360; throttle--) { //slowin motors to no rotation pwm value
            update(throttle, motorStruct, timeStruct, &pid);
        }
        motorStruct->wasThereFlight = 1; //security check
        resetPid();
        resetMotors();
    }
}


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    impCalibTst();
    Servo motorsArray[4] = {motors.upperLeftMotor, motors.upperRightMotor, motors.downLeftMotor, motors.downRightMotor};
    int motorPins[4] = {motorUpLeftConstant, motorUpRightConstant, motorDownLeftConstant, motorDownRightConstant};
    for (int i = 0; i < 4; i++) {
        motorsArray[i].attach(motorPins[i], minMicrosecondsConst, maxMicrosecondsConst);
        delay(1000);    // Delay to let the servo start correctly
    }
    resetMotors();  // Reset motors to 1500us
    Serial.begin(9600);
    delay(2000);    // Delay to start the calibration
    calibMpu(&mpu); // Calibrate MPU
    delay(100); // Delay to start the reading
}


void loop() {
    fly(&motors, &time);  // Fly function
}
