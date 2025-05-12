#include <Wire.h>
#include <Servo.h>
#define PI 3.14159265358979323846
#define MOTORDR 11   //Down Right motor
#define MOTORDL 10   //Down Left motor
#define MOTORUL 9    //Upper Left motor
#define MOTORUR 3    //Upper Right motor

/*
Ulepszenie kodu:
1) dodanie gain cosnatnt do motor pwm koncowe zeby troche wzmocnic  - ok
1') dodanie calib   - ok
2) dodanie funcji update regulujacej predkosci motorow 
3) dodanie funkcj ifly wykorzystujacej funkcje update
4) dodanie 2 funkcji: reset motors i resetPid kiedy zmienna wasThereFlight ustawiona na 1.
*/

//motor vars
Servo upperLeftMotor;
Servo downLeftMotor;
Servo upperRightMotor;
Servo downRightMotor;
int inputYaw;
int inputPitch;
int inputRoll;
int inputThrottle = 1000;
int throttleCutOff = 1000;
int motorsActivated = 0;
float upperLeftPwm, downLeftPwm, upperRightPwm, downRightPwm;
float gainConst = 1.2;
//pid vars
float rollPid, rollError, rollPrevError;    //vars for roll pid
float rollPTerm = 0;    //initial values
float rollITerm = 0;
float rollDTerm = 0;
double rollPConstant = 0.7; //3.55
double rollIConstant = 0.006;   //0.003
double rollDConstant = 1.2; //2.05
float desiredXAngle = 0;    //desired roll angle 0 to stabilise
float pitchPid, pitchError, pitchPrevError; //vars for pitch pid
float pitchPTerm = 0;   //initial values
float pitchITerm = 0;
float pitchDTerm = 0;
double pitchPConstant = 0.72;   //3.55
double pitchIConstant = 0.006;  //0.003
double pitchDConstant = 1.22;   //2.05
float desiredYAngle = 0;    //desired pitch angle
//time vars
unsigned long timeCalib;
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


void resetPid(void) {   //used for resettin pid after completed fly
    rollPrevError = 0; pitchPrevError = 0;
    rollITerm = 0; rollDTerm = 0; rollPTerm = 0;
    pitchITerm = 0; pitchDTerm = 0; pitchPTerm = 0;
}


void resetMotors(void) {
    upperLeftMotor.writeMicroseconds(throttleCutOff);
    upperRightMotor..writeMicroseconds(throttleCutOff);
    downLeftMotor.writeMicroseconds(throttleCutOff);
    downRightMotor.writeMicroseconds(throttleCutOff);
}


void calib(void) {
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
    if(accCalib == 0) { //calibration for accelerometer
        for(int a = 0; a < 2000; a++) {
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
    if(rateCalib == 0) {    //calibration for gyro
        for(int i = 0; i < 2000; i++) {
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


void update(int throttleValue) {

}


void fly(void) {

}


void setup() {
    upperLeftMotor.attach(MOTORUL); //left front motor
    downLeftMotor.attach(MOTORDL); //left back motor
    upperRightMotor.attach(MOTORUR); //right front motor 
    downRightMotor.attach(MOTORDR); //right back motor
    upperLeftMotor.writeMicroseconds(1000); 
    downLeftMotor.writeMicroseconds(1000);
    upperRightMotor.writeMicroseconds(1000); 
    downRightMotor.writeMicroseconds(1000);
    motorsActivated = 1;

    calib();    //gy521 calibration

    Serial.begin(9600);
    time = millis();
    timeCalib = millis();
}


void loop() {
    timePrev = time;
    time = millis();    //one loop iteration time used for integration
    elapsedTime = (time - timePrev) / 1000;   //ms -> s : 1 / 1000

    Wire.beginTransmission(0x68);   //startin communication for gyro
    Wire.write(0x43);   //gyro register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    rateY = Wire.read() << 8 | Wire.read(); //readin y first beacause of mpu placement on drone
    rateX = Wire.read() << 8 | Wire.read();
    rateX = (rateX / 32.8) - rateXError;    //32.8 corresponding 1000dps value
    rateY = (rateY / 32.8) - rateYError;  //convertin to deg/s
    angleXRate = rateX * elapsedTime;   //takin discrete for of integral for angle
    angleYRate = rateY * elapsedTime;

    Wire.beginTransmission(0x68);   //startin communication for acc
    Wire.write(0x3B);   //acc register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    accY = (Wire.read() << 8 | Wire.read()) / 4096.0;   //readin y first beacause of mpu placement on drone
    accX = (Wire.read() << 8 | Wire.read()) / 4096.0;  //converting to m/s^2
    accZ = (Wire.read() << 8 | Wire.read()) / 4096.0; 
    angleXAcc = (atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * radToDeg) - accAngleXError;
    angleYAcc = (atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * radToDeg) - accAngleYError;    

    angleX = 0.98 * (angleX + angleXRate) + 0.02 * angleXAcc;  //calculatin inclination angles usin complementary filter
    angleY = 0.98 * (angleY + angleYRate) + 0.02 * angleYAcc;
   
    desiredXAngle = 0;   //pid desired angles of inclination we would normally inputh here values from communication modules 
    desiredYAngle = 0;
    rollError = angleX - desiredXAngle;  //difference between current angle and desired one
    pitchError = angleY - desiredYAngle;
    rollPTerm = rollPConstant * rollError; //proportional terms of roll and pitch
    pitchPTerm = pitchPConstant * pitchError;
    if(-3 < rollError <3) { 
        rollITerm = rollITerm + (rollIConstant * rollError);    //integral term to smooth little error bet. -3deg:3deg
    }
    if(-3 < pitchError <3) {
        pitchITerm = pitchITerm + (pitchIConstant * pitchError);  
    }
    rollDTerm = rollDConstant * ((rollError - rollPrevError) / elapsedTime);   //derivative term = DConst*(dE/dt)
    pitchDTerm = pitchDConstant * ((pitchError - pitchPrevError) / elapsedTime);
    rollPid = rollPTerm + rollITerm + rollDTerm;   //final "calib" values from pid
    pitchPid = pitchPTerm + pitchITerm + pitchDTerm;
    if(rollPid < -400) {
        rollPid =- 400;   //security check to not exceed min/max pwm
    }
    if(rollPid > 400) {
        rollPid = 400; 
    }
    if(pitchPid < -400) {
        pitchPid =- 400;
    }
    if(pitchPid > 400) {
        pitchPid = 400;
    }

    upperRightPwm = gainConst * (115 + inputThrottle - rollPid - pitchPid);   //motors pwm calculation
    downRightPwm = gainConst * (115 + inputThrottle - rollPid + pitchPid);
    downLeftPwm = gainConst * (115 + inputThrottle + rollPid + pitchPid);
    upperLeftPwm = gainConst * (115 + inputThrottle + rollPid - pitchPid);
    if(upperRightPwm < 1100) {  //security check to not exceed min/max pwm
        upperRightPwm = 1100;
    }
    if(upperRightPwm > 2000) {
        upperRightPwm = 2000;
    }
    if(upperLeftPwm < 1100) {
        upperLeftPwm = 1100;
    }
    if(upperLeftPwm > 2000) {
        upperLeftPwm = 2000;
    }
    if(downRightPwm < 1100)
    {
        downRightPwm = 1100;
    }
    if(downRightPwm > 2000) {
        downRightPwm = 2000;
    }
    if(downLeftPwm < 1100) {
        downLeftPwm = 1100;
    }
    if(downLeftPwm > 2000) {
        downLeftPwm = 2000;
    }

    rollPrevError = rollError; //storin prev errors for roll and pitch
    pitchPrevError = pitchError;

    if(motorsActivated) {
        upperLeftMotor.writeMicroseconds(upperLeftPwm); 
        downLeftMotor.writeMicroseconds(downLeftPwm);
        upperRightMotor.writeMicroseconds(upperRightPwm); 
        downRightMotor.writeMicroseconds(downRightPwm);
    }
    if(!motorsActivated)
    {
        upperLeftMotor.writeMicroseconds(1000); 
        downLeftMotor.writeMicroseconds(1000);
        upperRightMotor.writeMicroseconds(1000); 
        downRightMotor.writeMicroseconds(1000);
    }

    if(millis() - timeCalib > 20000) {
        motorsActivated = 0;
    }

    Serial.println("UR, DR: " + String(upperRightPwm) + " " + String(downRightPwm) + "UL, DL: " + String(upperLeftPwm) + " " + String(downLeftPwm));
}