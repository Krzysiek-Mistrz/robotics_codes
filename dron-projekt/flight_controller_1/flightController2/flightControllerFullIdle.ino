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
Servo ESC11, ESC10, ESC9, ESC3; //pin<->motor: 11=UPPER_RIGHT=motor1, 10=UPPER_LEFT=motor4, 9=DOWN_LEFT=motor3, 3=DOWN_RIGHT=motor2
int MotorPWM = 1500;  //*MAY NEED CALIB*
unsigned long Time;
bool wasThereFlight = 0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
//gyro and acc variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
//predicted angles
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
//initialize output of filter
float Kalman1DOutput[] = {0, 0};
//desired rates and angles
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float DesiredAngleRoll, DesiredAnglePitch;
//pid variables *inner and outer PID*
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.6, PRatePitch = PRateRoll, PRateYaw = 2;
float IRateRoll = 3.5, IRatePitch = IRateRoll, IRateYaw = 12;
float DRateRoll = 0.03, DRatePitch = DRateRoll, DRateYaw = 0;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2, PAnglePitch = PAngleRoll;
float IAngleRoll = 0, IAnglePitch = IAngleRoll;
float DAngleRoll = 0, DAnglePitch = DAngleRoll;

//*MAY NEED CALIB*
/*
void fly(float MotorInput1, float MotorInput2, float MotorInput3, float MotorInput4){
  //real: 80 max duszenie; 2 - min duszenie
  //imag: max duszenie dla 90, min dla 0 v 180
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
    ESC11.writeMicroseconds(i - MotorInput1);
    ESC10.writeMicroseconds(i - MotorInput2);
    ESC9.writeMicroseconds(i + MotorInput3);
    ESC3.writeMicroseconds(i + MotorInput4);
    delay(10);
  }
  MotorPWM = 1800;
  //fly up until less than 10s
  while (Time < 10000UL) {
    ESC11.writeMicroseconds(MotorPWM - MotorInput1);
    ESC10.writeMicroseconds(MotorPWM - MotorInput2);
    ESC9.writeMicroseconds(MotorPWM + MotorInput3);
    ESC3.writeMicroseconds(MotorPWM + MotorInput4);
    Time = millis();
  }
  //decreasing throttle
  for (int i = MotorPWM; i > 1500; i--){
    ESC11.write(i - MotorInput1);
    ESC10.write(i - MotorInput2);
    ESC9.write(i + MotorInput3);
    ESC3.write(i + MotorInput4);
    delay(50);
  }
  wasThereFlight = 1;
}
*/

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  //kalman equations
  KalmanState = KalmanState + 0.004 * KalmanInput; 
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1*KalmanUncertainty + 3*3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;
  //output
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
  /*
  kalman input = rotation rate
  kalmanMeasurement = acc angle
  kalmanState = angle calculated
  */
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  //readin acc
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  //readin gyro
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  //calculatin gyro
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  //calculatin acc
  AccX=(float)AccXLSB/4096-0.05;
  AccY=(float)AccYLSB/4096+0.01;
  AccZ=(float)AccZLSB/4096-0.11;
  //end angles relative to OX and OY
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ* 
 AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*
 AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  //calculatin p, i, d terms of pid
  //p term
  float Pterm = P * Error;
  //i term
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400)
    Iterm = 400;
  else if (Iterm < -400)
    Iterm = -400;
  //d term
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
    PIDOutput = 400;
  else if (PIDOutput < -400)
    PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  //resetting rates (inner pid)
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
  //resetting angles (outer pid)
  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
}

//*MAY NEED CALIB*
void update(void) {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  //calculatin kalman angles for roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  //Serial.print("Roll Angle [°] ");
  //Serial.print(KalmanAngleRoll);
  //Serial.print(" Pitch Angle [°] ");
  //Serial.println(KalmanAnglePitch);
  DesiredAnglePitch = DesiredAngleRoll = 0; //dont wanna rotate around y and x axis accordingly
  //InputThrottle = 90; here for sure change!!!
  DesiredRateYaw = 0; //dont wanna rotate around z axis
  //calculatin errors for angles
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  //outer pid for angles roll and pitch
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];
  //calculatin errors for change rates for angles
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  //inner pid for rates roll pitch and yaw (OX, OY, OZ accordingly)
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];
  //calculatin motor input
  MotorInput1 = (InputRoll + InputPitch + InputYaw);
  MotorInput2 = (InputRoll - InputPitch - InputYaw);
  MotorInput3 = (InputRoll + InputPitch - InputYaw);
  MotorInput4 = (InputRoll - InputPitch + InputYaw);
}

void setup() {
  Serial.begin(9600);
  //esc init
  ESC11.attach(ESCPIN11, MinMicroseconds, MaxMicroseconds);
  ESC10.attach(ESCPIN10, MinMicroseconds, MaxMicroseconds);
  ESC9.attach(ESCPIN9, MinMicroseconds, MaxMicroseconds);
  ESC3.attach(ESCPIN3, MinMicroseconds, MaxMicroseconds);
  //led indicator
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //comm with gyro and calib
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //mpu 6050 calib
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  LoopTimer = micros();
}

void loop() {
  update();
  Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput3) + " " + String(MotorInput4));
  //*MAY NEED CALIB*
  /*because goin to the function with only current pid inputroll, inputpitch and inputyaw variables and the function works only on this few variables without knowing the current state needs fix. You would need to get the current state of those variables in teh function*/
  /*
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
      update();
      Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput4));
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
      update();
      ESC11.writeMicroseconds(MotorPWM);// - MotorInput1);
      ESC3.writeMicroseconds(MotorPWM);// - MotorInput2);
      ESC9.writeMicroseconds(MotorPWM);// + MotorInput3);
      ESC10.writeMicroseconds(MotorPWM);// + MotorInput4);
      Time = millis();
    }
    //decreasing throttle
    for (int i = MotorPWM; i > 1500; i--){
      update();
      Serial.println("MOTOR INPUTS 1, 2, 3, 4: " + String(MotorInput1) + " " + String(MotorInput2) + " " + String(MotorInput4));
      ESC11.writeMicroseconds(i);// - MotorInput1);
      ESC3.writeMicroseconds(i);// - MotorInput2);
      ESC9.writeMicroseconds(i);// + MotorInput3);
      ESC10.writeMicroseconds(i);// + MotorInput4);
      delay(50);
    }
    wasThereFlight = 1;
  }
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
  */
}