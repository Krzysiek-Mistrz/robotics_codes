#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>
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
int throttleCutOff = 1500;
int maxFlyable = 1800;  //+140 from pid, = 1800
int minFlyable = 1650;  //+140 from pid, = 1650
float upperLeftPwm = 0.0f, downLeftPwm = 0.0f, upperRightPwm = 0.0f, downRightPwm = 0.0f;
float upperLeftPwmFinal = 0.0f, downLeftPwmFinal = 0.0f, upperRightPwmFinal = 0.0f, downRightPwmFinal = 0.0f;
float gainConst = 1.0f;
bool wasThereFlight = 0;
//pid vars
float rollPid = 0.0f, rollError = 0.0f, rollPrevError = 0.0f;    //vars for roll pid
float rollPTerm = 0.0f;    //initial values
float rollITerm = 0.0f;
float rollDTerm = 0.0f;
float rollPConstant = 0.7f; //3.55
float rollIConstant = 0.006f;   //0.003
float rollDConstant = 1.2f; //2.05
float desiredXAngle = 0.0f;    //desired roll angle 0 to stabilise
float pitchPid = 0.0f, pitchError = 0.0f, pitchPrevError = 0.0f; //vars for pitch pid
float pitchPTerm = 0.0f;   //initial values
float pitchITerm = 0.0f;
float pitchDTerm = 0.0f;
float pitchPConstant = 0.72f;   //3.55
float pitchIConstant = 0.006f;  //0.003
float pitchDConstant = 1.22f;   //2.05
float desiredYAngle = 0.0f;    //desired pitch angle
float yawPid = 0.0f, yawError = 0.0f, yawPrevError = 0.0f; //vars for pitch pid
float yawPTerm = 0.0f;   //initial values
float yawITerm = 0.0f;
float yawDTerm = 0.0f;
float yawPConstant = 0.72f;   //3.55
float yawIConstant = 0.006f;  //0.003
float yawDConstant = 1.22f;   //2.05
float desiredZAngle = 0.0f;    //desired yaw angle
//time vars
float time = 0.0f;
float timePrev = 0.0f;
float elapsedTime = 0.0f;
unsigned long flyTime = 0UL;
//GY521 vars
MPU6050 mpu(Wire);
double angleX = 0.0;
double angleY = 0.0;
double angleZ = 0.0;


void impCalibTst(void) {
    Wire.begin();
    byte status = mpu.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    while(status != 0) { 
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}


void resetPid(void) {   //used for resettin pid after completed fly
    rollPrevError = 0; pitchPrevError = 0;
    rollITerm = 0; rollDTerm = 0; rollPTerm = 0;
    pitchITerm = 0; pitchDTerm = 0; pitchPTerm = 0;
    rollPid = 0; pitchPid = 0; yawPid = 0;
}


void resetMotors(void) {  //used to reset motors after flight
    upperLeftMotor.writeMicroseconds(throttleCutOff);
    upperRightMotor.writeMicroseconds(throttleCutOff);
    downLeftMotor.writeMicroseconds(throttleCutOff);
    downRightMotor.writeMicroseconds(throttleCutOff);
}


void anglesRead(void) {
    mpu.update();
    angleX = mpu.getAngleY();  //calculatin inclination angles usin complementary filter
    angleY = mpu.getAngleX(); //changing order cause of mpu placement
    angleZ = mpu.getAngleZ();
}


void pidEquation(void) {
    desiredXAngle = 0;   //pid desired angles of inclination we would normally inputh here values from communication modules 
    desiredYAngle = 0;
    desiredZAngle = 0;
    rollError = angleX - desiredXAngle;  //difference between current angle and desired one
    pitchError = angleY - desiredYAngle;
    yawError = angleZ - desiredZAngle;
    rollPTerm = rollPConstant * rollError; //proportional terms of roll and pitch
    pitchPTerm = pitchPConstant * pitchError;
    yawPTerm = yawPConstant * yawError;
    rollITerm = rollITerm + (rollIConstant * (rollError + rollPrevError) * (elapsedTime / 2));    //integral term to smooth little error bet. -3deg:3deg
    pitchITerm = pitchITerm + (pitchIConstant * (pitchError + pitchPrevError) * (elapsedTime / 2));
    yawITerm = yawITerm + (yawIConstant * (yawError + yawPrevError) * (elapsedTime / 2));
    if(rollITerm > 400)   rollITerm = 400;
    else if(rollITerm < -400)   rollITerm = -400;
    if(pitchITerm > 400)   pitchITerm = 400;
    else if(pitchITerm < -400)   pitchITerm = -400;
    if(yawITerm > 400)   yawITerm = 400;
    else if(yawITerm < -400)   yawITerm = -400;
    rollDTerm = rollDConstant * ((rollError - rollPrevError) / elapsedTime);   //derivative term = DConst*(dE/dt)
    pitchDTerm = pitchDConstant * ((pitchError - pitchPrevError) / elapsedTime);
    yawDTerm = yawDConstant * ((yawError - yawPrevError) / elapsedTime);
    rollPid = rollPTerm + rollITerm + rollDTerm;   //final "calib" values from pid
    pitchPid = pitchPTerm + pitchITerm + pitchDTerm;
    yawPid = yawPTerm + yawITerm + yawDTerm;
    if(rollPid < -400) {
        rollPid = -400;   //security check to not exceed min/max pwm
    }
    if(rollPid > 400) {
        rollPid = 400; 
    }
    if(pitchPid < -400) {
        pitchPid = -400;
    }
    if(pitchPid > 400) {
        pitchPid = 400;
    }
    if(yawPid < -400) {
        yawPid = -400;
    }
    if(yawPid > 400) {
        yawPid = 400;
    }
}


void update(int throttleValue) {
    timePrev = time;
    time = millis();    //one loop iteration time used for integration
    elapsedTime = (time - timePrev) / 1000;   //ms -> s : 1 / 1000
    upperRightPwm = gainConst * (-rollPid - pitchPid - yawPid);   //motors pwm calculation
    downRightPwm = gainConst * (-rollPid + pitchPid + yawPid);
    downLeftPwm = gainConst * (rollPid + pitchPid - yawPid);
    upperLeftPwm = gainConst * (rollPid - pitchPid + yawPid);
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
    rollPrevError = rollError; //storin prev errors for roll and pitch
    pitchPrevError = pitchError;
    yawPrevError = yawError;
    upperLeftMotor.writeMicroseconds(upperLeftPwmFinal); 
    downLeftMotor.writeMicroseconds(downLeftPwmFinal);
    upperRightMotor.writeMicroseconds(upperRightPwmFinal); 
    downRightMotor.writeMicroseconds(downRightPwmFinal);
    Serial.println("UR, DR: " + String(upperRightPwmFinal) + " " + String(downRightPwmFinal) + " UL, DL: " + String(upperLeftPwmFinal) + " " + String(downLeftPwmFinal));
}


void fly(void) {
    if (wasThereFlight == 0) {
        for (int i = throttleCutOff; i < maxFlyable; i++) { //increasing throttle from 1500
            update(i);
        }
        flyTime = millis();
        while (millis() - flyTime < 5000UL) { //fly up until less than 5s
            anglesRead(); //readin angles
            pidEquation();  //readin final pid for roll and pitch angles
            update(maxFlyable);
        }
        rollPid = 0; pitchPid = 0; yawPid = 0;
        for (int i = maxFlyable; i > minFlyable; i--) { //decreasing throttle
            update(i);
        }
        rollPid = 0; pitchPid = 0; yawPid = 0;
        flyTime = millis();
        while (millis() - flyTime < 20000UL) { //falling
            anglesRead(); //readin angles
            pidEquation();  //readin final pid for roll and pitch angles
            update(minFlyable);
        }
        rollPid = 0; pitchPid = 0; yawPid = 0;
        for (int i = minFlyable; i > throttleCutOff; i--) { //slowin motors to no rotation pwm value
            update(i);
        }
        wasThereFlight = 1; //security check
        resetPid();
        resetMotors();
    }
}


void setup() {
    impCalibTst();
    upperLeftMotor.attach(MOTORUL, MINMICROSECONDS, MAXMICROSECONDS); //left front motor
    downLeftMotor.attach(MOTORDL, MINMICROSECONDS, MAXMICROSECONDS); //left back motor
    upperRightMotor.attach(MOTORUR, MINMICROSECONDS, MAXMICROSECONDS); //right front motor
    downRightMotor.attach(MOTORDR, MINMICROSECONDS, MAXMICROSECONDS); //right back motor
    delay(250); //for esc conf
    mpu.calcOffsets();  //gy521 calibration
    time = millis();
    delay(250); //for 1st dt
    Serial.begin(9600);
}


void loop() {
    fly();
}