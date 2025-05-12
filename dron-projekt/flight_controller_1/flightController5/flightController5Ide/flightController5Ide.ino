#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

#define MOTOR2 11   //DR motor
#define MOTOR3 10   //DL motor
#define MOTOR4 9    //UL motor
#define MOTOR1 3    //UR motor


//motor vars
Servo motor1, motor2, motor3, motor4;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
int MotorPWM = 1500;  //calibrated for 2480kv frequency/voltage=kv, 0deg/s = 1500us
int ThrottleIdle = 1180;    //(18% power)
int MinMicroseconds = 1000; 
int MaxMicroseconds = 2000;
float GainConstant = 5;
//gy521 vars
Adafruit_MPU6050 mpu;
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;
//time length of each loop in us
uint32_t LoopTimer;
//pid vars
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;  //desired value - curr value
float InputRoll, InputThrottle, InputPitch, InputYaw; // 0deg/s, x, 0deg/s, 0deg/s
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;  //prev errors for next iter
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;  //prev iterms for next iter
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.6 ; float PRatePitch = PRateRoll; float PRateYaw = 2; //constants for proportional term
float IRateRoll = 3.5 ; float IRatePitch = IRateRoll; float IRateYaw = 12;  //constants for integration term
float DRateRoll = 0.03 ; float DRatePitch = DRateRoll; float DRateYaw = 0;  //constants for derivative term


void gyroSignals(void) {
  //Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //printin the values of gyro rad/s of gyro rot. x = rate roll, rot. y = rate pitch, rot. z = rate yaw *DEBUG*
  //Serial.println(String("RATE ROLL, RATE PITCH, RATE YAW: ") + " " + String(g.gyro.x) + " " + String(g.gyro.y) + " " + String(g.gyro.z));
  RateRoll = (float)g.gyro.y * GainConstant;
  RatePitch = (float)g.gyro.x * GainConstant;
  RateYaw = (float)g.gyro.z * GainConstant;
  //for our program we are takin gyro.y for roll because mpu6050 rotated by pi/2
}


void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
    //proportional term
    float Pterm = P * Error;
    //integrating term
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;  //dt = 0.004s
    //checking for integral windup error (accumulation of errors)
    if (Iterm > 400)    Iterm = 400;
    else if (Iterm < -400)  Iterm =- 400;
    //derivative term
    float Dterm = D * (Error - PrevError) / 0.004;
    //input formula: input = P * Error + D * (Error - PrevError) / 0.004 + (PrevIterm + I * (Error + PrevError) * 0.004 / 2)
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)    PIDOutput = 400;
    else if (PIDOutput <- 400)   PIDOutput =- 400;
    //storing current error for next PrevError and Iterm for next PrevIterm
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}


void reset_pid(void) {
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
}


void update(int ThrottleValue) {
    //gettin current RateRoll, RatePitch, ... and substr average value from calib
    gyroSignals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    //0deg/s -> 1500us 75deg/s -> 2000 -75deg/s -> 1000us 1500 in the middle
    DesiredRateRoll = 0.15 * (1500 - 1500);
    DesiredRatePitch = 0.15 * (1500 - 1500);
    DesiredRateYaw = 0.15 * (1500 - 1500);
    //how much 'power' to motors
    InputThrottle = ThrottleValue;
    //calculatin errors: desired value - curr value
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
    
    //calculatin needed Roll input for motors
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll = PIDReturn[0];
        PrevErrorRateRoll = PIDReturn[1]; 
        PrevItermRateRoll = PIDReturn[2];
    //calculatin needed Pitch input for motors
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch = PIDReturn[0]; 
        PrevErrorRatePitch = PIDReturn[1]; 
        PrevItermRatePitch = PIDReturn[2];
    //calculatin needed Yaw input for motors
    pid_equation(ErrorRateYaw, PRateYaw,
        IRateYaw, DRateYaw, PrevErrorRateYaw,
        PrevItermRateYaw);
        InputYaw = PIDReturn[0]; 
        PrevErrorRateYaw = PIDReturn[1]; 
        PrevItermRateYaw = PIDReturn[2];

    //limitin input throttle because stabilisation of roll, pitch, yaw needed
    if (InputThrottle > 1800) InputThrottle = 1800;
    //calculatin motor inputs
    MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw) - 35;
    MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw) - 35;
    MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw) - 35;
    MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw) - 35;
    //checking for overloadin (100%)
    if (MotorInput1 > 2000)   MotorInput1 = 1999;
    if (MotorInput2 > 2000)   MotorInput2 = 1999; 
    if (MotorInput3 > 2000)   MotorInput3 = 1999; 
    if (MotorInput4 > 2000)   MotorInput4 = 1999;
    //checking for minimum power requirement (18%)
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    //writing calculated pwm to motors
    /*
    analogWrite(MOTOR1, MotorInput1);
    analogWrite(MOTOR2, MotorInput2);
    analogWrite(MOTOR3, MotorInput3);
    analogWrite(MOTOR4, MotorInput4);
    */
    motor1.writeMicroseconds(MotorInput1);
    motor2.writeMicroseconds(MotorInput2);
    motor3.writeMicroseconds(MotorInput3);
    motor4.writeMicroseconds(MotorInput4);

    Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput3) + " " + String(MotorInput4));
    //waitin for next iteration (4000us) (250Hz)
    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();
}

void setup() {
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    //definin motor setup
    motor1.attach(MOTOR1, MinMicroseconds, MaxMicroseconds);
    motor2.attach(MOTOR2, MinMicroseconds, MaxMicroseconds);
    motor3.attach(MOTOR3, MinMicroseconds, MaxMicroseconds);
    motor4.attach(MOTOR4, MinMicroseconds, MaxMicroseconds);

    //checkin if mpu connected
    if (!mpu.begin()) {
        Serial.println("ERROR: FAILED TO FIND GY521");
        while (1) {
            delay(10);
        }
    }
    //setting max measured range of acceleration -8G to 8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    //setting max measured range of acceleration -500deg/s to 500deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //settin range of passed frequency - LOW PASS FILTER
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    delay(100);

    //calibratin mpu6050
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        gyroSignals();
        //Serial.println(RatePitch);
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    //setting average value
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    
    LoopTimer = micros();
}


void loop() {
    update(MotorPWM);
    //delay(500);
}