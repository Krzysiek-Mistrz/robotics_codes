#include <Wire.h>
TwoWire Wire2(2, I2C_FAST_MODE); // inicjalizacja I2C2 (piny B10 - SCL, B11 - SDA (domyslnie wire obsluguje I2C1)

// ustawienia pid dla roll
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;
int pid_max_roll = 400;
// ustawienia pid dla pitch
float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;
// ustawienia pid dla yaw
float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;
// odpowiada za wlaczenie kontroli poziomu do drona
boolean auto_level = true;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
//do standaryzacji setpointu dla pidu
float roll_level_adjust, pitch_level_adjust;

// ustawienie wartosci kalibracyjnych manualnych dla temp ~25*C
uint8_t use_manual_calibration = true;
int16_t manual_acc_pitch_cal_value = 115;
int16_t manual_acc_roll_cal_value = -78;
int16_t manual_gyro_pitch_cal_value = -212;
int16_t manual_gyro_roll_cal_value = 504;
int16_t manual_gyro_yaw_cal_value = -45;
// adres gy521
uint8_t gyro_address = 0x68;
// zmienne  kalibracyjne oraz do acc i gyro
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

// zmienne pomocnicze do sygnalizacji stanow drona
uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t highByte, lowByte, start;
uint8_t error, error_counter, error_led;
float battery_voltage;

// pwm do esc i zmienne do motors
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int;

// zmienne do receivera
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

// do kontrolowania czasu obiegu kazdej iteracji
uint32_t loop_timer, error_timer;



// do ustawienia stanu na diodzie (sygnalizacja stanu drona) DO IMPL !!!NA PLYTCE!!!
void red_led(int8_t level) {
  digitalWrite(PB4, level);
}

void green_led(int8_t level) {
  digitalWrite(PB3, level);
}



// funkcja pid dzialajaca w przerwaniach
void calculate_pid(void){
  // obliczenie dla czesci integrujacej, prop, roznicz roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)
    pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)
    pid_i_mem_roll = pid_max_roll * -1;
  // output dla roll KpEp + KiEi + KdEd
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)
    pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)
    pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  // obliczenie dla czesci integrujacej, prop, roznicz pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;
  // output dla pitch KpEp + KiEi + KdEd
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  // obliczenie dla czesci integrujacej, prop, roznicz yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)
    pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)
    pid_i_mem_yaw = pid_max_yaw * -1;
  // output dla yaw KpEp + KiEi + KdEd
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
}



// funkcja pid dzialajaca w przerwaniach
void calibrate_gyro(void) {
  // przeskakujemy kalibracje gdy mamy wartosci doswiadczalne ;)
  if (use_manual_calibration)
    cal_int = 2000;
  else {
    cal_int = 0;
    manual_gyro_pitch_cal_value = 0;
    manual_gyro_roll_cal_value = 0;
    manual_gyro_yaw_cal_value = 0;
  }
  //jezeli ustawilismy kalib automatyczna (podstawy drona musza byc !!!IDEALNIE ROWNE!!! -> do poprawy) 
  if (cal_int != 2000) {
    //probkujemy (2000d) dane z gyro
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {
      if (cal_int % 25 == 0) 
        digitalWrite(PB4, !digitalRead(PB4));
      gyro_signals();
      gyro_roll_cal += gyro_roll;
      gyro_pitch_cal += gyro_pitch;
      gyro_yaw_cal += gyro_yaw;
      delay(4);
    }
    red_led(HIGH);
    //bierzemy srednia artmetyczna z danych w celu uzyskania wart z kalibracji AVR be like
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
    manual_gyro_pitch_cal_value = gyro_pitch_cal;
    manual_gyro_roll_cal_value = gyro_roll_cal;
    manual_gyro_yaw_cal_value = gyro_yaw_cal;
  }
}



// funkcja do pokazania sygnalu bledu
void error_signal(void){
  if (error >= 100) 
    red_led(HIGH);
  //sprawdzenie czy minelo >= 250ms i stanu error'ow
  else if (error_timer < millis()){
    error_timer = millis() + 250;
    //do kodow bledow kazdy blad ma inny kodzik ;)
    if(error > 0 && error_counter > error + 3) 
      error_counter = 0;
    if (error_counter < error && error_led == 0 && error > 0){
      red_led(HIGH);
      error_led = 1;
    }
    // wylaczamy diode i zwiekszamy counter bledow o 1
    else{
      red_led(LOW);
      error_counter++;
      error_led = 0;
    }
  }
}



// funkcja do nawiazywania polaczenia z gy521
void gyro_setup(void){
  // nawiazujemy polaczenie z gy521 i je aktywujemy
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x6B);
  Wire2.write(0x00);
  Wire2.endTransmission();
  // ustawiamy skale 500dps
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1B);
  Wire2.write(0x08);
  Wire2.endTransmission();
  // ustawiamy skale (+/-)8g
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1C);
  Wire2.write(0x10);
  Wire2.endTransmission();
  // ustawiamy lpf w celu wyeliminowania zaklocen wysokiego pasma (tzn. ok. 42Hz)
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1A);
  Wire2.write(0x03);
  Wire2.endTransmission();
}



//funkcja do obslugi receivera (channel 1)
void handler_channel_1(void) {
  //jezeli stan na A0 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR  >> 0) {
    channel_1_start = TIMER2_BASE->CCR1;
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;
    if (channel_1 < 0)
      channel_1 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}
//funkcja do obslugi receivera (channel 2)
void handler_channel_2(void) {
  //jezeli stan na A1 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR >> 1) {
    channel_2_start = TIMER2_BASE->CCR2;
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;
    if (channel_2 < 0)
      channel_2 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}
//funkcja do obslugi receivera (channel 3)
void handler_channel_3(void) {
  //jezeli stan na A2 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR >> 2) {
    channel_3_start = TIMER2_BASE->CCR3;
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;
    if (channel_3 < 0)
      channel_3 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}
//funkcja do obslugi receivera (channel 4)
void handler_channel_4(void) {
  //jezeli stan na A3 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR >> 3) {
    channel_4_start = TIMER2_BASE->CCR4;
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;
    if (channel_4 < 0)
      channel_4 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}
//funkcja do obslugi receivera (channel 5)
void handler_channel_5(void) {
  //jezeli stan na A4 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR >> 6) {
    channel_5_start = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;
    if (channel_5 < 0)
      channel_5 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}
//funkcja do obslugi receivera (channel 6)
void handler_channel_6(void) {
  //jezeli stan na A5 jest HIGH -> zczytujemy czas RISE_EDGE
  if (0b1 & GPIOA_BASE->IDR >> 7) {
    channel_6_start = TIMER3_BASE->CCR2;
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;
  }
  //obliczamy calkowita dlugosc trwania pulsu RISE_EDGE-FALL_EDGE
  else {
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;
    if (channel_6 < 0)
      channel_6 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}



// odczyt gyro i acc z mpu6050 (gy521)
void gyro_signals(void) {
  //ropoczynamy komunikacje a pozniej zaczynamy czytac...
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x3B);
  Wire2.endTransmission();
  Wire2.requestFrom(gyro_address, 14);
  //shiftujemy w lewo bity zeby starczylo na kolejne dane
  acc_y = Wire2.read() << 8 | Wire2.read();
  acc_x = Wire2.read() << 8 | Wire2.read();
  acc_z = Wire2.read() << 8 | Wire2.read();
  temperature = Wire2.read() << 8 | Wire2.read();
  gyro_roll = Wire2.read() << 8 | Wire2.read();
  gyro_pitch = Wire2.read() << 8 | Wire2.read();
  gyro_yaw = Wire2.read() << 8 | Wire2.read();
  gyro_pitch *= -1;
  gyro_yaw *= -1;
  //odejmujemy od wartosci odczytaj wartosc manualnej (gdy mamy wlaczona) lub automatyczna (przypisana do manulanej ;))
  acc_y -= manual_acc_pitch_cal_value;
  acc_x -= manual_acc_roll_cal_value;
  gyro_roll -= manual_gyro_roll_cal_value;
  gyro_pitch -= manual_gyro_pitch_cal_value;
  gyro_yaw -= manual_gyro_yaw_cal_value;
}



// timer do obslugi przerwan
void timer_setup(void) {
  // przerwania do handlera 1, 2, 3, 4
  Timer2.attachCompare1Interrupt(handler_channel_1);
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001;
  TIMER2_BASE->CCMR2 = 0b100000001;
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;
  // przerwania do handlera 5, 6
  Timer3.attachCompare1Interrupt(handler_channel_5);
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001;
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;
  //wlaczenie timera 4 i dla niego przerwan
  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR1 = 1000;
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}



// bateria, gyro, kalibracja gyro
void setup() {
  //batt
  pinMode(4, INPUT_ANALOG);
  
  // umozliwia uzycie pb3 i pb4 jako i/o pin
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  //na pb3
  green_led(LOW);
  // na pb4
  red_led(HIGH);

  // do debugu
  //Serial.begin(115200);
  //delay(250);

  // setup timerow do przerwan
  timer_setup();
  delay(50);

  // sprawdzenie poprawnosci komunikacji z mpu6050
  Wire2.begin();
  Wire2.beginTransmission(gyro_address);
  error = Wire2.endTransmission();
  while (error != 0) {
    //kod bledu komunikacji z gy521 jest 2
    error = 2;
    error_signal();
    delay(4);
  }

  // setup do zyroskopu (kalibracja zyroskopu)
  gyro_setup();
  //jezeli kalibracja automatyczna -> wywolanie sekwencji sygnalizacyjnej
  if (!use_manual_calibration) {
    for (count_var = 0; count_var < 1250; count_var++) {
      if (count_var % 125 == 0) {
        digitalWrite(PB4, !digitalRead(PB4));
      }
      delay(4);
    }
    count_var = 0;
  }
  //kalibracja zyroskpu
  calibrate_gyro();

  //nieprawidlowe sygnaly transmittera sygnaly bledu
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    //kod bledu 3
    error = 3;
    error_signal();
    delay(4);
  }
  error = 0;

  //koniecznosc ustawienia throttle w pozycji poczatkowej
  while (channel_3 < 990 || channel_3 > 1050) {
    //gdy nie mamy ustawienia poczatkowego -> blad
    error = 4;
    error_signal();
    delay(4);
  }
  error = 0;

  //wylaczamy diode kiedy wszystkie procedury wykonane
  red_led(LOW);

  // 0 - 0V ; 4095 - 36.3V (11*3.3, poniewaz divider 1/11)
  battery_voltage = (float)analogRead(4) / 112.81;

  //wlaczamy timer do kontroli czasu kazdej iteracji
  loop_timer = micros();

  //sygnalizacja gotowosci do startu
  green_led(HIGH);
}



void loop() {
  // pokazujmey sygnaly ew bledow
  error_signal();

  // do zczytania acc, gyro, temp ustandaryzowanych za pomoca calib values
  gyro_signals();

  // konwersja na deg/s (65.5 = 1 deg/sec) wartosci do pid
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);
  // kat obliczany z gyro (kat dryfu) 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;
  // poprawka kata o wychylenie yaw
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);

  // obliczenie katow z przyspieszenia
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  // katy obliczane z przyspieszen dla kolejno pitch i roll
  if (abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  //filterek komplementarny
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  // doswiadczalna poprawka katow
  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;
  // jezeli nie chcemy by poprawka byla stosowana do autolevel'owania
  if (!auto_level) {
    pitch_level_adjust = 0;
    roll_level_adjust = 0;
  }


  // start silnikow: throttle LOW ; yaw LEFT
  if (channel_3 < 1050 && channel_4 < 1050)
    start = 1;
  // kiedy yaw > centrum (czyli kiedy joystick puścisz) startujemy silniki
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
    start = 2;
    
    // sygnal - wystartowalismy
    green_led(LOW);

    // na starcie kat z acc = katowi z gyro (niwelujemy rozbieznosci)
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    // resetujemy tez wartosci do integral i d z pid zeby nie zaburzac startu
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  // zatrzymanie silnikow: throttle LOW ; yaw RIGHT
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;
    // sygnal gotowosci do startu
    green_led(HIGH);
  }

  // setpoint dla roll
  pid_roll_setpoint = 0;
  // setpoint do pida dla pitch, roll, yaw zalezy od wartosci poszczegolnych wartosci z receivera (pasmo 1508 - 1492 jest martwym pasmem dodajemy je do poprawy setpointa)
  if (channel_1 > 1508)
    pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)
    pid_roll_setpoint = channel_1 - 1492;
  //obliczamy setpoint odejmujac wartosc korekcji kata
  pid_roll_setpoint -= roll_level_adjust;
  //zeby otrzymac wartosc ustandaryzowana setpoint'a w deg/s
  pid_roll_setpoint /= 3.0;
  
  // setpoint dla pitch
  pid_pitch_setpoint = 0;
  // setpoint do pida dla pitch, roll, yaw zalezy od wartosci poszczegolnych wartosci z receivera (pasmo 1508 - 1492 jest martwym pasmem dodajemy je do poprawy setpointa)
  if (channel_2 > 1508)
    //odwrocenie pitch NOWE
    pid_pitch_setpoint = -channel_2 + 1508;
  else if (channel_2 < 1492)
    pid_pitch_setpoint = -channel_2 + 1492;
  //obliczamy setpoint odejmujac wartosc korekcji kata
  pid_pitch_setpoint -= pitch_level_adjust;
  //zeby otrzymac wartosc ustandaryzowana setpoint'a w deg/s
  pid_pitch_setpoint /= 3.0;

  // setpoint dla yaw
  pid_yaw_setpoint = 0;
  // setpoint do pida dla pitch, roll, yaw zalezy od wartosci poszczegolnych wartosci z receivera (pasmo 1508 - 1492 jest martwym pasmem dodajemy je do poprawy setpointa)
  if (channel_3 > 1050) {
    if (channel_4 > 1508)
      //od razu ze standaryzacja /3
      pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)
      pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  //obliczenie wartosci outputu z pid
  calculate_pid();

  // obliczenie U na batt z pomoca filtra komplementarnego (doswiadczalne wyznaczenie)
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //jezeli U < 10V daj kod bledu
  if (battery_voltage < 10.0 && error == 0)
    error = 1;

  //throttle na kanale 3 odczytywane bezposrednio
  throttle = channel_3;

  // gdy gotowy do startu
  if (start == 2) {
    // 200 miejsca na pid i usera
    if (throttle > 1800) throttle = 1800;
    // obliczenie pwm na esc 1, 2, 3, 4
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
    // nie przekraczamy wartosci granicznych
    if (esc_1 < 1100)
      esc_1 = 1100;
    if (esc_2 < 1100)
      esc_2 = 1100;
    if (esc_3 < 1100)
      esc_3 = 1100;
    if (esc_4 < 1100)
      esc_4 = 1100;
    // nie przekraczamy wartosci granicznych
    if (esc_1 > 2000)
      esc_1 = 2000;
    if (esc_2 > 2000)
      esc_2 = 2000;
    if (esc_3 > 2000)
      esc_3 = 2000;
    if (esc_4 > 2000)
      esc_4 = 2000;
  }
  // jezeli nie start utrzymuj staly pwm 1000 na silnikach
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  // podanie wartosci throttle z receivera na esc 1, 2, 3, 4
  TIMER4_BASE->CCR1 = esc_1;
  TIMER4_BASE->CCR2 = esc_2;
  TIMER4_BASE->CCR3 = esc_3;
  TIMER4_BASE->CCR4 = esc_4;
  //resetujemy timer 4
  TIMER4_BASE->CNT = 5000;
  
  // obieg petli musi wynosic dokladnie 4000us jak nie to wystawiamy sygnal bledu
  if (micros() - loop_timer > 4050)
    error = 5;
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}
