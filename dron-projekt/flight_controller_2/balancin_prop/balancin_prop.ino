#include <Wire.h>

TwoWire Wire2(2, I2C_FAST_MODE); // Inicjalizacja I2C2 (piny B10 - SCL, B11 - SDA)

int32_t vibration_array[20], avarage_vibration_level, vibration_total_result;
uint8_t array_counter, throttle_init_ok, vibration_counter;
uint32_t wait_timer;

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;

//Let's declare some variables so we can use them in the complete program.
uint8_t disable_throttle, flip32;
uint32_t loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
int16_t loop_counter;
uint8_t data = 1;
uint8_t start, warning;
int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

//The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
uint8_t gyro_address = 0x68;


void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;

  if (warning == 0) {
    TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
    TIMER4_BASE->CR2 = 0;
    TIMER4_BASE->SMCR = 0;
    TIMER4_BASE->DIER = 0;
    TIMER4_BASE->EGR = 0;
    TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
    TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
    TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
    TIMER4_BASE->PSC = 71;
    TIMER4_BASE->ARR = 4000;
    TIMER4_BASE->DCR = 0;
    TIMER4_BASE->CCR1 = 1000;

    TIMER4_BASE->CCR1 = channel_3;
    TIMER4_BASE->CCR2 = channel_3;
    TIMER4_BASE->CCR3 = channel_3;
    TIMER4_BASE->CCR4 = channel_3;
    pinMode(PB6, PWM);
    pinMode(PB7, PWM);
    pinMode(PB8, PWM);
    pinMode(PB9, PWM);
  }
}



void handler_channel_1(void) {                           //This function is called when channel 1 is captured.
  if (0b1 & GPIOA_BASE->IDR  >> 0) {                     //If the receiver channel 1 input pulse on A0 is high.
    channel_1_start = TIMER2_BASE->CCR1;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 1 input pulse on A0 is low.
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;     //Calculate the total pulse time.
    if (channel_1 < 0)channel_1 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
  }
}
void handler_channel_2(void) {                           //This function is called when channel 2 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 1) {                      //If the receiver channel 2 input pulse on A1 is high.
    channel_2_start = TIMER2_BASE->CCR2;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 2 input pulse on A1 is low.
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;     //Calculate the total pulse time.
    if (channel_2 < 0)channel_2 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
  }
}
void handler_channel_3(void) {                           //This function is called when channel 3 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 2) {                      //If the receiver channel 3 input pulse on A2 is high.
    channel_3_start = TIMER2_BASE->CCR3;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 3 input pulse on A2 is low.
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;     //Calculate the total pulse time.
    if (channel_3 < 0)channel_3 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;               //Change the input capture mode to the rising edge of the pulse.
  }
}
void handler_channel_4(void) {                           //This function is called when channel 4 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 3) {                      //If the receiver channel 4 input pulse on A3 is high.
    channel_4_start = TIMER2_BASE->CCR4;                 //Record the start time of the pulse.
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 4 input pulse on A3 is low.
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;     //Calculate the total pulse time.
    if (channel_4 < 0)channel_4 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;               //Change the input capture mode to the rising edge of the pulse.
  }
}
void handler_channel_5(void) {                           //This function is called when channel 5 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 6) {                      //If the receiver channel 5 input pulse on A6 is high.
    channel_5_start = TIMER3_BASE->CCR1;                 //Record the start time of the pulse.
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 5 input pulse on A6 is low.
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;     //Calculate the total pulse time.
    if (channel_5 < 0)channel_5 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;               //Change the input capture mode to the rising edge of the pulse.
  }
}
void handler_channel_6(void) {                           //This function is called when channel 6 is captured.
  if (0b1 & GPIOA_BASE->IDR >> 7) {                      //If the receiver channel 6 input pulse on A7 is high.
    channel_6_start = TIMER3_BASE->CCR2;                 //Record the start time of the pulse.
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;                //Change the input capture mode to the falling edge of the pulse.
  }
  else {                                                 //If the receiver channel 6 input pulse on A7 is low.
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;     //Calculate the total pulse time.
    if (channel_6 < 0)channel_6 += 0xFFFF;               //If the timer has rolled over a correction is needed.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;               //Change the input capture mode to the rising edge of the pulse.
  }
}


void setup() {
  pinMode(4, INPUT_ANALOG);
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.

  pinMode(PB3, INPUT);                                         //Set PB3 as input.
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) || digitalRead(PB3)) flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
  else flip32 = 0;

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  Serial.begin(115200);                                         //Set the serial output to 115200 kbps.
  delay(100);                                                   //Give the serial port some time to start to prevent data loss.
  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.

  Serial.println("Setup complete!");                            // Debug: inform that setup is complete.
}

void loop() {
  delay(10);

  loop_timer = micros() + 4000;                                                        
  gyro_signalen();                                                                 //Get the raw gyro and accelerometer data.
  //Calculate the total accelerometer vector.
  vibration_array[0] = sqrt((acc_axis[1] * acc_axis[1]) + (acc_axis[2] * acc_axis[2]) + (acc_axis[3] * acc_axis[3]));

  avarage_vibration_level = 0; // Initialize avarage_vibration_level
  for (array_counter = 16; array_counter > 0; array_counter--) {                   //Do this loop 16 times to create an array of accelrometer vectors.
    vibration_array[array_counter] = vibration_array[array_counter - 1];           //Shift every variable one position up in the array.
    avarage_vibration_level += vibration_array[array_counter];                     //Add the array value to the acc_av_vector variable.
  }
  avarage_vibration_level /= 17;                                                   //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

  if (vibration_counter < 20) {                                                    //If the vibration_counter is less than 20 do this.
    vibration_counter ++;                                                          //Increment the vibration_counter variable.
    vibration_total_result += abs(vibration_array[0] - avarage_vibration_level);   //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
  } else {
    vibration_counter = 0;                                                         //If the vibration_counter is equal or larger than 20 do this.
    Serial.println(vibration_total_result / 50);                                   //Print the total accelerometer vector divided by 50 on the serial monitor.
    vibration_total_result = 0;                                                    //Reset the vibration_total_result variable.
  }

  if (data == '1') {                                                               //If the user requested 1.
    TIMER4_BASE->CCR1 = channel_3;                                                 //Set the ESC 1 output pulse equal to the throttle input.
    TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
    TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
    TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
  }
  if (data == '2') {                                                               //If the user requested 2.
    TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
    TIMER4_BASE->CCR2 = channel_3;                                                 //Set the ESC 2 output pulse equal to the throttle input.
    TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
    TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
  }
  if (data == '3') {                                                               //If the user requested 3.
    TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
    TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
    TIMER4_BASE->CCR3 = channel_3;                                                 //Set the ESC 3 output pulse equal to the throttle input.
    TIMER4_BASE->CCR4 = 1000;                                                      //Keep the ESC 4 pulse at 1000us.
  }
  if (data == '4') {                                                               //If the user requested 4.
    TIMER4_BASE->CCR1 = 1000;                                                      //Keep the ESC 1 pulse at 1000us.
    TIMER4_BASE->CCR2 = 1000;                                                      //Keep the ESC 2 pulse at 1000us.
    TIMER4_BASE->CCR3 = 1000;                                                      //Keep the ESC 3 pulse at 1000us.
    TIMER4_BASE->CCR4 = channel_3;                                                 //Set the ESC 4 output pulse equal to the throttle input.
  }
  if (data == '5') {                                                               //If the user requested 5.
    TIMER4_BASE->CCR1 = channel_3;                                                 //Set the ESC 1 output pulse equal to the throttle input.
    TIMER4_BASE->CCR2 = channel_3;                                                 //Set the ESC 2 output pulse equal to the throttle input.
    TIMER4_BASE->CCR3 = channel_3;                                                 //Set the ESC 3 output pulse equal to the throttle input.
    TIMER4_BASE->CCR4 = channel_3;                                                 //Set the ESC 4 output pulse equal to the throttle input.
  }
  
  while (loop_timer > micros()); //Wait for the loop time to be reached
  
  Serial.println("Loop iteration complete."); // Debug: inform that loop iteration is complete.
}

void gyro_signalen() {
  //Read the MPU-6050 data.
  Wire2.beginTransmission(gyro_address);                       //Start communication with the gyro.
  Wire2.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  Wire2.endTransmission();                                     //End the transmission.
  Wire2.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.

  acc_axis[1] = Wire2.read() << 8 | Wire2.read();              //Add the low and high byte to the acc_x variable.
  acc_axis[2] = Wire2.read() << 8 | Wire2.read();              //Add the low and high byte to the acc_y variable.
  acc_axis[3] = Wire2.read() << 8 | Wire2.read();              //Add the low and high byte to the acc_z variable.
  temperature = Wire2.read() << 8 | Wire2.read();              //Add the low and high byte to the temperature variable.
  gyro_axis[1] = Wire2.read() << 8 | Wire2.read();             //Add the low and high byte to the gyro_x variable.
  gyro_axis[2] = Wire2.read() << 8 | Wire2.read();             //Add the low and high byte to the gyro_y variable.
  gyro_axis[3] = Wire2.read() << 8 | Wire2.read();             //Add the low and high byte to the gyro_z variable.

  if (cal_int == 2000) {                                        //If the calibration counter is at 2000 do the following.
    gyro_axis_cal[1] = gyro_axis[1];                            //Set the gyro_axis_cal variable to the average calibration value.
    gyro_axis_cal[2] = gyro_axis[2];                            //Set the gyro_axis_cal variable to the average calibration value.
    gyro_axis_cal[3] = gyro_axis[3];                            //Set the gyro_axis_cal variable to the average calibration value.
    acc_axis_cal[1] = acc_axis[1];                              //Set the acc_axis_cal variable to the average calibration value.
    acc_axis_cal[2] = acc_axis[2];                              //Set the acc_axis_cal variable to the average calibration value.
    acc_axis_cal[3] = acc_axis[3];                              //Set the acc_axis_cal variable to the average calibration value.
  }

  //Subtract the manual accelerometer calibration value.
  acc_axis[1] -= manual_acc_pitch_cal_value;
  acc_axis[2] -= manual_acc_roll_cal_value;
  //Subtract the manual gyro calibration value.
  gyro_axis[1] -= manual_gyro_pitch_cal_value;
  gyro_axis[2] -= manual_gyro_roll_cal_value;
  gyro_axis[3] -= manual_gyro_yaw_cal_value;
  //Subtract the gyro calibration value.
  gyro_axis[1] -= gyro_axis_cal[1];
  gyro_axis[2] -= gyro_axis_cal[2];
  gyro_axis[3] -= gyro_axis_cal[3];
  //Subtract the accelerometer calibration value.
  acc_axis[1] -= acc_axis_cal[1];
  acc_axis[2] -= acc_axis_cal[2];
  acc_axis[3] -= acc_axis_cal[3];
}
