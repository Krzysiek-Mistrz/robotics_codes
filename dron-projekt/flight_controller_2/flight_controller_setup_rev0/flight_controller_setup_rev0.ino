#include <Wire.h>

TwoWire Wire2(2, I2C_FAST_MODE); // Inicjalizacja I2C2 (piny B10 - SCL, B11 - SDA)

//zmienne do gyro
uint8_t use_manual_calibration = true;
int16_t manual_acc_pitch_cal_value = 115
int16_t manual_acc_roll_cal_value = -78
int16_t manual_gyro_pitch_cal_value = -212
int16_t manual_gyro_roll_cal_value = 504
int16_t manual_gyro_yaw_cal_value = -45

int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
uint8_t gyro_address = 0x68;

//zmienne do komunikacji z trsnmitterem
uint8_t disable_throttle;
uint8_t data, start, warning;
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

//do bateri
float battery_voltage;

//do czasu
int16_t loop_counter;
uint32_t loop_timer;



//do wyprintowania intro
void print_intro(void) {
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("          YMFC-32 quadcopter setup tool"));
  Serial.println(F("==================================================="));
  Serial.println(F("a = Read the receiver input pulses"));
  Serial.println(F("b = I2C scanner to detect any I2C sensors attached"));
  Serial.println(F("c = Read the raw gyro values"));
  Serial.println(F("d = Read the raw accelerometer values"));
  Serial.println(F("e = Check the IMU angles"));
  Serial.println(F("f = Test the LEDs"));
  Serial.println(F("g = Read the battery voltage input"));
  Serial.println(F("h = Get gyro and accelerometer calibration values"));
  Serial.println(F("==================================================="));
  Serial.println(F("1 = Check motor 1 (front right, counter clockwise direction)"));
  Serial.println(F("2 = Check motor 2 (rear right, clockwise direction)"));
  Serial.println(F("3 = Check motor 3 (rear left, counter clockwise direction)"));
  Serial.println(F("4 = Check motor 4 (front left, clockwise direction)"));
  Serial.println(F("5 = Check all motors"));
  Serial.println(F("==================================================="));
  Serial.println(F("For support and questions: www.brokking.net"));
  Serial.println(F(""));
  if (!disable_throttle) {                                      //If the throttle is not disabled.
    Serial.println(F("==================================================="));
    Serial.println(F("     WARNING >>>THROTTLE IS ENABLED<<< WARNING"));
    Serial.println(F("==================================================="));
  }
  //delay(5)  //sprawdzic!
}



//dow kalibracji i wyswietlania wartosci gyro i acc
void read_gyro_values(void) {
  //kiedy ustawiona manualna kalibracja to ustawiamy zmienna skipujaca
  if (use_manual_calibration)
    cal_int = 2000;
  //w przeciwnym razie robimy kalib automatyczna
  else {
    cal_int = 0;
    manual_gyro_pitch_cal_value = 0;
    manual_gyro_roll_cal_value = 0;
    manual_gyro_yaw_cal_value = 0;
  }
  //dopoki user nie wprowadzi q pozostan w petli
  while (data != 'q') {
    delay(250);
    if (Serial.available() > 0) {
      data = Serial.read();
      delay(100);
      while (Serial.available() > 0)
        loop_counter = Serial.read();
    }
    //kalibracja zyroskopu
    if (data == 'c') {
      if (cal_int != 2000) {
        gyro_axis_cal[1] = 0;
        gyro_axis_cal[2] = 0;
        gyro_axis_cal[3] = 0;
        Serial.print("Calibrating the gyro");
        //zbieramy 2000 probek do kalibracji
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {
          if (cal_int % 125 == 0) {
            digitalWrite(PB3, !digitalRead(PB3));
            Serial.print(".");
          }
          //wywolujemy funckje wywolujaca pomiary nastepnie dodajemy do zmiennych kalibracyjnych i standaryzujemy dzielac przez 2000
          gyro_signals();
          gyro_axis_cal[1] += gyro_axis[1];
          gyro_axis_cal[2] += gyro_axis[2];
          gyro_axis_cal[3] += gyro_axis[3];
          //wszytsko w obiegu 4000us
          delay(4);
        }
        Serial.println(".");
        //standaryzacja
        gyro_axis_cal[1] /= 2000;
        gyro_axis_cal[2] /= 2000;
        gyro_axis_cal[3] /= 2000;
        //podmianka z manual (to pozniej manual wykorzystujemy :)
        manual_gyro_pitch_cal_value = gyro_axis_cal[2];
        manual_gyro_roll_cal_value = gyro_axis_cal[1];
        manual_gyro_yaw_cal_value = gyro_axis_cal[3];
      }
      gyro_signals();
      Serial.print("Gyro_x = ");
      Serial.print(gyro_axis[1]);
      Serial.print(" Gyro_y = ");
      Serial.print(gyro_axis[2]);
      Serial.print(" Gyro_z = ");
      Serial.println(gyro_axis[3]);
    }
    else {
      gyro_signals();
      Serial.print("ACC_x = ");
      Serial.print(acc_axis[1]);
      Serial.print(" ACC_y = ");
      Serial.print(acc_axis[2]);
      Serial.print(" ACC_z = ");
      Serial.println(acc_axis[3]);
    }
  }
  print_intro();
}



//odczyt danych z receiver'a
void reading_receiver_signals(void) {
  while (data != 'q') {
    delay(250);
    if (Serial.available() > 0) {
      data = Serial.read();
      delay(100);
      while (Serial.available() > 0)loop_counter = Serial.read();
    }
    //jezeli throttle - low i yaw - left rozpoczynamy proces startu 
    if (channel_3 < 1100 && channel_4 < 1100)
      start = 1;
    //jezeli throttle - low i yaw - center rozpoczynamy nadawanie pwm do motorow (start)
    if (start == 1 && channel_3 < 1100 && channel_4 > 1450)
      start = 2;
    //jezeli throttle - low i yaw - right konczymy nadawanie pwm do esc -> motorow :) 
    if (start == 2 && channel_3 < 1100 && channel_4 > 1900)
      start = 0;

    Serial.print("Start:");
    Serial.print(start);

    //wydruk wartosci odebieranych z channel1 -> roll
    Serial.print("  Roll:");
    if (channel_1 - 1480 < 0)
      Serial.print("<<<");
    else if (channel_1 - 1520 > 0)
      Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(channel_1);

    //wydruk wartosci odebieranych z channel2 -> pitch
    Serial.print("  Pitch:");
    if (channel_2 - 1480 < 0)
      Serial.print("^^^");
    else if (channel_2 - 1520 > 0)
      Serial.print("vvv");
    else 
      Serial.print("-+-");
    Serial.print(channel_2);

    //wydruk wartosci odebieranych z channel3 -> throttle
    Serial.print("  Throttle:");
    if (channel_3 - 1480 < 0)
      Serial.print("vvv");
    else if (channel_3 - 1520 > 0)
      Serial.print("^^^");
    else 
      Serial.print("-+-");
    Serial.print(channel_3);

    //wydruk wartosci odbieranych z channel4 -> yaw
    Serial.print("  Yaw:");
    if (channel_4 - 1480 < 0)
      Serial.print("<<<");
    else if (channel_4 - 1520 > 0)
      Serial.print(">>>");
    else 
      Serial.print("-+-");
    Serial.print(channel_4);

    //pzst kanal 5 jakbym chcial cos dodac w przyszlosci
    Serial.print("  CH5:");
    Serial.print(channel_5);

    //pzst kanal 6 jakbym chcial cos dodac w przyszlosci
    Serial.print("  CH6:");
    Serial.println(channel_6);

  }
  print_intro();
}



//funkcja do testowania ledow
void test_leds(void) {
  data = 0;
  if (Serial.available() > 0) {
    data = Serial.read();
    delay(100);
    while (Serial.available() > 0)
      loop_counter = Serial.read();
  }
  Serial.println(F("The red LED is now ON for 3 seconds"));
  red_led(HIGH);
  delay(3000);
  Serial.println(F(""));
  Serial.println(F("The green LED is now ON for 3 seconds"));
  red_led(LOW);
  green_led(HIGH);
  delay(3000);
  green_led(LOW);
  print_intro();
}



//funkcja do obslugi timerow (w tym przerwan od poszczegolnych kanalow nadawczych)
void timer_setup(void) {
  //do odbioru w przerwaniach od transmittera (1, 2, 3, 4)
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
  
  //do odbioru w przerwaniach z 5, 6
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

  loop_counter = 0;
  //sprawdzenie czy input z transmittera jest wgl poprawny
  while ((channel_3 > 2100 || channel_3 < 900) && warning == 0) {
    delay(100);
    loop_counter++;
    if (loop_counter == 40) {
      Serial.println(F("Waiting for a valid receiver channel-3 input signal"));
      Serial.println(F(""));
      Serial.println(F("The input pulse should be between 1000 till 2000us"));
      Serial.print(F("Current channel-3 receiver input value = "));
      Serial.println(channel_3);
      Serial.println(F(""));
      Serial.println(F("Is the receiver connected and the transmitter on?"));
      Serial.println(F("For more support and questions: www.brokking.net"));
      Serial.println(F(""));
      Serial.print(F("Waiting for another 5 seconds."));
    }
    if (loop_counter > 40 && loop_counter % 10 == 0)
      Serial.print(F("."));
    //po 90 iteracjach wylacza silniki dla bezpieczenstwa (gdy caly czas zly podawany sygnal)
    if (loop_counter == 90) {
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("The ESC outputs are disabled for safety!"));
      warning = 1;
    }
  }
  //timer 4 do generowania sygnalow pwm na wyjsciach odpowiednich kanalow
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

    //sygnal z channel3 reguluje wartosc pwm podawanego z pinow kanalow
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



//do zczytania napiecia na baterii
void check_battery_voltage(void) {
  loop_counter = 0;
  battery_voltage = analogRead(4);
  while (data != 'q') {
    delayMicroseconds(4000);
    if (Serial.available() > 0) {
      data = Serial.read();
      delay(100);
      while (Serial.available() > 0)
        loop_counter = Serial.read();
    }
    loop_counter++;
    //every one second 250 * 4000us = 1s
    if (loop_counter == 250) {
      Serial.print("Voltage = ");
      Serial.print(battery_voltage / 112.81, 1);
      Serial.println("V");
      loop_counter = 0;
    }
    //uzywamy filtera complementarnego by wywalic spiki w sygnale napiecia powodowane przez esc
    battery_voltage = (battery_voltage * 0.99) + ((float)analogRead(4) * 0.01);
  }
  loop_counter = 0;
  print_intro();
}



//obliczamy katy na podstawie gyro i acc
void check_imu_angles(void) {
  uint8_t first_angle = 0;
  loop_counter = 0;
  first_angle = false;
  //sprawdzamy czy zmienna do kalib manualnej ustawiona
  if (use_manual_calibration)
    cal_int = 2000;
  //jezeli nie to zerujemy wartosci manualne
  else {
    cal_int = 0;
    manual_gyro_pitch_cal_value = 0;
    manual_gyro_roll_cal_value = 0;
    manual_gyro_yaw_cal_value = 0;
  }
  while (data != 'q') {
    loop_timer = micros() + 4000;
    if (Serial.available() > 0) {
      data = Serial.read();
      delay(100);
      while (Serial.available() > 0)loop_counter = Serial.read();
    }
    //szybka kalibracja jezeli ustawiono automatyczna kalib
    if (cal_int == 0) {
      gyro_axis_cal[1] = 0;
      gyro_axis_cal[2] = 0;
      gyro_axis_cal[3] = 0;
      Serial.print("Calibrating the gyro");
      for (cal_int = 0; cal_int < 2000 ; cal_int ++) {
        if (cal_int % 125 == 0) {
          digitalWrite(PB3, !digitalRead(PB3));
          Serial.print(".");
        }
        gyro_signals();
        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        delay(4);
      }
      Serial.println(".");
      green_led(LOW);
      gyro_axis_cal[1] /= 2000;
      gyro_axis_cal[2] /= 2000;
      gyro_axis_cal[3] /= 2000;

      manual_gyro_pitch_cal_value = gyro_axis_cal[2];
      manual_gyro_roll_cal_value = gyro_axis_cal[1];
      manual_gyro_yaw_cal_value = gyro_axis_cal[3];
    }

    gyro_signals();

    //liczenie roll i pitch angles z gyro 0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_axis[2] * 0.0000611;                                            //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += gyro_axis[1] * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

    //poprawka roll i pitch o yaw 0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
    angle_pitch -= angle_roll * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin(gyro_axis[3] * 0.000001066);                        //If the IMU has yawed transfer the pitch angle to the roll angel.

    //sprawdzanie zakresu wartosci przyspieszenia
    if (acc_axis[1] > 4096)
      acc_axis[1] = 4096;
    if (acc_axis[1] < -4096)
      acc_axis[1] = -4096;
    if (acc_axis[2] > 4096)
      acc_axis[2] = 4096;
    if (acc_axis[2] < -4096)
      acc_axis[2] = -4096;

    //liczenie roll i pitch angles z acc 57.296 = 1 / (3.142 / 180)
    angle_pitch_acc = asin((float)acc_axis[1] / 4096) * 57.296;
    angle_roll_acc = asin((float)acc_axis[2] / 4096) * 57.296;

    //jezeli pierwszy raz uruchomienie to nalezy przypisac wartosci 0 kata roll i pitch do wartosci kata z acc
    if (!first_angle) {
      angle_pitch = angle_pitch_acc;
      angle_roll = angle_roll_acc;
      first_angle = true;
    }
    //zastosowanie filtru komplementarnego do obliczenia wartosci angle pitch i angle roll
    else {
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }

    //nie wypisujemy wszystiego naraz (ze wzg mozliwe bledy)
    if (loop_counter == 0)
      Serial.print("Pitch: ");
    if (loop_counter == 1)
      Serial.print(angle_pitch , 1);
    if (loop_counter == 2)
      Serial.print(" Roll: ");
    if (loop_counter == 3)
      Serial.print(angle_roll , 1);
    if (loop_counter == 4)
      Serial.print(" Yaw: ");
    if (loop_counter == 5)
      Serial.print(gyro_axis[3] / 65.5 , 0);
    if (loop_counter == 6)
      Serial.print(" Temp: ");
    if (loop_counter == 7)
      Serial.println(temperature / 340.0 + 35.0 , 1);
    loop_counter ++;
    if (loop_counter == 60)
      loop_counter = 0;

    while (loop_timer > micros());
  }
  loop_counter = 0;
  print_intro();
}



//funkcja sprawdzajaca wibracje motorow (do propellers balancing)
void check_motor_vibrations(void) {
  int32_t vibration_array[20], avarage_vibration_level, vibration_total_result;
  uint8_t array_counter, throttle_init_ok, vibration_counter;
  uint32_t wait_timer;
  throttle_init_ok = 0;
  while (data != 'q') {
    loop_timer = micros() + 4000;
    if (Serial.available() > 0) {
      data = Serial.read();
      delay(100);
      while (Serial.available() > 0)
        loop_counter = Serial.read();
    }

    if (throttle_init_ok) {
      gyro_signals();
      //obliczenie calkowitego wektora przyspieszenia dzialajacego na imu
      vibration_array[0] = sqrt((acc_axis[1] * acc_axis[1]) + (acc_axis[2] * acc_axis[2]) + (acc_axis[3] * acc_axis[3]));

      for (array_counter = 16; array_counter > 0; array_counter--) {
        vibration_array[array_counter] = vibration_array[array_counter - 1];
        avarage_vibration_level += vibration_array[array_counter];
      }
      //srednia wektora przyspieszenia
      avarage_vibration_level /= 17;

      //mierzymy poziom wibracji jako roznice wartosci wektora przyspienia calk i sredniego wektora przyspieszenia
      if (vibration_counter < 20) {
        vibration_counter ++;
        vibration_total_result += abs(vibration_array[0] - avarage_vibration_level);
      }
      else {
        vibration_counter = 0;
        Serial.println(vibration_total_result / 50);
        vibration_total_result = 0;
      }

      //ustawiamy pwm = throttle value do esc1 (TIMER4 OBSLUGUJE PRZERWANIE PODAWANIA PWM)
      if (data == '1') {
        TIMER4_BASE->CCR1 = channel_3;
        TIMER4_BASE->CCR2 = 1000;
        TIMER4_BASE->CCR3 = 1000;
        TIMER4_BASE->CCR4 = 1000;
      }
      //ustawiamy pwm = throttle value do esc2
      if (data == '2') {
        TIMER4_BASE->CCR1 = 1000;
        TIMER4_BASE->CCR2 = channel_3;
        TIMER4_BASE->CCR3 = 1000;
        TIMER4_BASE->CCR4 = 1000;
      }
      //ustawiamy pwm = throttle value do esc3
      if (data == '3') {
        TIMER4_BASE->CCR1 = 1000;
        TIMER4_BASE->CCR2 = 1000;
        TIMER4_BASE->CCR3 = channel_3;
        TIMER4_BASE->CCR4 = 1000;
      }
      //ustawiamy pwm = throttle value do esc4
      if (data == '4') {
        TIMER4_BASE->CCR1 = 1000;
        TIMER4_BASE->CCR2 = 1000;
        TIMER4_BASE->CCR3 = 1000;
        TIMER4_BASE->CCR4 = channel_3;
      }
      //ustawiamy pwm = throttle value do esc4, 3, 2, 1 (wszystkie motory uruchomic)
      if (data == '5') {
        TIMER4_BASE->CCR1 = channel_3;
        TIMER4_BASE->CCR2 = channel_3;
        TIMER4_BASE->CCR3 = channel_3;
        TIMER4_BASE->CCR4 = channel_3;
      }
    }
    //dla throttle w najnizszej pozycji
    else {
      wait_timer = millis() + 10000;
      //kiedy wyzej najnizszej pozycji wysiwtelamy wartosc wykryta pwm
      if (channel_3 > 1050) {
        Serial.println(F("Throttle is not in the lowest position."));
        Serial.print(F("Throttle value is: "));
        Serial.println(channel_3);
        Serial.print(F(""));
        Serial.print(F("Waiting for 10 seconds:"));
      }
      //tak dlugo jak w najnizszj pozycji
      while (wait_timer > millis() && !throttle_init_ok) {
        if (channel_3 < 1050)
          throttle_init_ok = 1;
        delay(500);
        Serial.print(F("."));
      }
    }
    if (!throttle_init_ok) {
      data = 'q';
    }
    while (loop_timer > micros());                                                     //250Hz -> 4000us czekamy
  }
  loop_counter = 0;
  print_intro();
}



//funkcja do wykrywania urzadzen i2c
void i2c_scanner(void) {
  data = 0;
  uint8_t error, address, done;
  uint16_t nDevices;
  Serial.println("Scanning address 1 till 127...");
  Serial.println("");
  nDevices = 0;
  //lecimy po wszystkich adresach i jak znajdzie to sygnalizacja po endTriansmission
  for (address = 1; address < 127; address++) {
    Wire2.beginTransmission(address);
    error = Wire2.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  done = 1;
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
  delay(2000);
  print_intro();
}



//funkcje obslugi przewan zglaszanych przez timery (odczyt danych z transmittera) (funkcja podawania na esc takze w przerwaniach ale uwzg wczesniej)
void handler_channel_1(void) {
  //jezeli sygnal na A0 jest HIGH
  if (0b1 & GPIOA_BASE->IDR  >> 0) {
    //zapisujemy czas poczatkowy 
    channel_1_start = TIMER2_BASE->CCR1;
    //zmieniamy tryb odczytu na input capture falling edge
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;
  }
  //jezeli sygnal na A0 jest LOW
  else {
    //obliczamy calkowity czas trwania pulsu
    channel_1 = TIMER2_BASE->CCR1 - channel_1_start;
    //poprawka gdy timer dobil do konca
    if (channel_1 < 0)
      channel_1 += 0xFFFF;
    //zmiana trybu odczytu na rising edge
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_2(void) {
  //jezeli sygnal na A1 jest HIGH
  if (0b1 & GPIOA_BASE->IDR >> 1) {
    channel_2_start = TIMER2_BASE->CCR2;
    TIMER2_BASE->CCER |= TIMER_CCER_CC2P;
  }
  //jezeli sygnal na A1 jest LOW
  else {
    channel_2 = TIMER2_BASE->CCR2 - channel_2_start;
    if (channel_2 < 0)
      channel_2 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}

void handler_channel_3(void) {
  //jezeli sygnal na A2 jest HIGH
  if (0b1 & GPIOA_BASE->IDR >> 2) {
    channel_3_start = TIMER2_BASE->CCR3;
    TIMER2_BASE->CCER |= TIMER_CCER_CC3P;
  }
  //jezeli sygnal na A2 jest LOW
  else {
    channel_3 = TIMER2_BASE->CCR3 - channel_3_start;
    if (channel_3 < 0)
      channel_3 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC3P;
  }
}

void handler_channel_4(void) {
  //jezeli sygnal na A3 jest HIGH
  if (0b1 & GPIOA_BASE->IDR >> 3) {
    channel_4_start = TIMER2_BASE->CCR4;
    TIMER2_BASE->CCER |= TIMER_CCER_CC4P;
  }
  //jezeli sygnal na A3 jest LOW
  else {
    channel_4 = TIMER2_BASE->CCR4 - channel_4_start;
    if (channel_4 < 0)
      channel_4 += 0xFFFF;
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC4P;
  }
}

void handler_channel_5(void) {
  //jezeli sygnal na A4 jest HIGH
  if (0b1 & GPIOA_BASE->IDR >> 6) {
    channel_5_start = TIMER3_BASE->CCR1;
    TIMER3_BASE->CCER |= TIMER_CCER_CC1P;
  }
  //jezeli sygnal na A4 jest LOW
  else {
    channel_5 = TIMER3_BASE->CCR1 - channel_5_start;
    if (channel_5 < 0)
      channel_5 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;
  }
}

void handler_channel_6(void) {
  //jezeli sygnal na A5 jest HIGH
  if (0b1 & GPIOA_BASE->IDR >> 7) {
    channel_6_start = TIMER3_BASE->CCR2;
    TIMER3_BASE->CCER |= TIMER_CCER_CC2P;
  }
  //jezeli sygnal na A5 jest LOW
  else {
    channel_6 = TIMER3_BASE->CCR2 - channel_6_start;
    if (channel_6 < 0)
      channel_6 += 0xFFFF;
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC2P;
  }
}



//funkcja do kalibracji imu manualnie (ustala wartosci do recznej kalibracji)
void manual_imu_calibration(void) {
  data = 0;
  acc_axis_cal[1] = 0;
  acc_axis_cal[2] = 0;
  gyro_axis_cal[1] = 0;
  gyro_axis_cal[2] = 0;
  gyro_axis_cal[3] = 0;
  Serial.print("Calibrating the gyro");
  //pobieramy dane z gyro do jego stablilizacji elektronicznej z sym 4000us
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0) {
      digitalWrite(PB3, !digitalRead(PB3));
      Serial.print(".");
    }
    gyro_signals();
    delay(4);
  }

  //liczymy wszytskie offsety do gyro i acc przez estymator EX (war. pocz: manualacc, ... = 0)
  for (cal_int = 0; cal_int < 4000 ; cal_int ++) {
    if (cal_int % 125 == 0) {
      digitalWrite(PB3, !digitalRead(PB3));
      Serial.print(".");
    }
    gyro_signals();
    acc_axis_cal[1] += acc_axis[1] + manual_acc_pitch_cal_value;
    acc_axis_cal[2] += acc_axis[2] + manual_acc_roll_cal_value;
    gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;
    gyro_axis_cal[2] += gyro_axis[2] + manual_gyro_pitch_cal_value;
    gyro_axis_cal[3] += gyro_axis[3] + manual_gyro_yaw_cal_value;
    delay(4);
  }
  Serial.println(".");
  green_led(LOW);
  //estymator
  acc_axis_cal[1] /= 4000;
  acc_axis_cal[2] /= 4000;
  gyro_axis_cal[1] /= 4000;
  gyro_axis_cal[2] /= 4000;
  gyro_axis_cal[3] /= 4000;

  //printujemy wartosci do kalibracji (ktorych bedziemy uzywac do kalibracji manualnej przy stalej temperaturze
  Serial.print("manual_acc_pitch_cal_value = ");
  Serial.println(acc_axis_cal[1]);
  Serial.print("manual_acc_roll_cal_value = ");
  Serial.println(acc_axis_cal[2]);
  Serial.print("manual_gyro_pitch_cal_value = ");
  Serial.println(gyro_axis_cal[2]);
  Serial.print("manual_gyro_roll_cal_value = ");
  Serial.println(gyro_axis_cal[1]);
  Serial.print("manual_gyro_yaw_cal_value = ");
  Serial.println(gyro_axis_cal[3]);

  print_intro();
}



//funkcja obslugujaca ledy
void red_led(int8_t level) {
    digitalWrite(PB4, level);
}
void green_led(int8_t level) {
    digitalWrite(PB3, level);
}



//setup do obslugi ledow, batt, wst komunikacji z mpu, interfejsu 
void setup() {
  pinMode(4, INPUT_ANALOG);
  //umozliwia ustawienie I/O 3, 4 w trybie output
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);

  green_led(LOW);
  red_led(LOW);

  Serial.begin(115200);
  delay(100);
  timer_setup();
  delay(50);
  
  //ustaiwnie stanu aktywnego
  Wire2.begin();
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x6B);
  Wire2.write(0x00);
  Wire2.endTransmission();
  //ustawienie dps
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1B);
  Wire2.write(0x08);
  Wire2.endTransmission();
  //ustawienie +/-8g
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1C);
  Wire2.write(0x10);
  Wire2.endTransmission();
  //ustawienie LPFilter 43Hz
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x1A);
  Wire2.write(0x03);
  Wire2.endTransmission();

  print_intro();
}



//glowna petla programu (wybor funkcji)
void loop() {
  delay(10);

  if (Serial.available() > 0) {
    data = Serial.read();
    delay(100);
    while (Serial.available() > 0)
      loop_counter = Serial.read();
    disable_throttle = 1;
  }

  //ustawiamy pwm do wsyztskich esc w przerwaniach kiedy zmienne bezpieczenstwa false
  if (!disable_throttle) {
    TIMER4_BASE->CCR1 = channel_3;
    TIMER4_BASE->CCR2 = channel_3;
    TIMER4_BASE->CCR3 = channel_3;
    TIMER4_BASE->CCR4 = channel_3;
  } else {
    TIMER4_BASE->CCR1 = 1000;
    TIMER4_BASE->CCR2 = 1000;
    TIMER4_BASE->CCR3 = 1000;
    TIMER4_BASE->CCR4 = 1000;
  }

  //wybieramy funkcje ktora chcemy sprawdzic
  if (data == 'a') {
    Serial.println(F("Reading receiver input pulses."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    reading_receiver_signals();
  }

  if (data == 'b') {
    Serial.println(F("Starting the I2C scanner."));
    i2c_scanner();
  }

  if (data == 'c') {
    Serial.println(F("Reading raw gyro data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    read_gyro_values();
  }

  if (data == 'd') {
    Serial.println(F("Reading the raw accelerometer data."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    read_gyro_values();
  }

  if (data == 'e') {
    Serial.println(F("Reading the IMU angles."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_imu_angles();
  }

  if (data == 'f') {
    Serial.println(F("Test the LEDs."));
    test_leds();
  }

  if (data == 'g') {
    Serial.println(F("Reading the battery voltage."));
    Serial.println(F("You can exit by sending a q (quit)."));
    check_battery_voltage();
  }

  if (data == 'h') {
    Serial.println(F("Get manual gyro and accelerometer calibration values."));
    manual_imu_calibration();
  }

  if (data == '1') {
    Serial.println(F("Check motor 1 (front right, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '2') {
    Serial.println(F("Check motor 2 (rear right, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '3') {
    Serial.println(F("Check motor 3 (rear left, counter clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '4') {
    Serial.println(F("Check motor 4 (front lefft, clockwise direction)."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }

  if (data == '5') {
    Serial.println(F("Check motor all motors."));
    Serial.println(F("You can exit by sending a q (quit)."));
    delay(2500);
    check_motor_vibrations();
  }
}



//funkcja do obslugi komunikacji z gyro
void gyro_signals(void) {
  //rozpocznij komunikacje z mpu -> zczytywanie z rejestrow
  Wire2.beginTransmission(gyro_address);
  Wire2.write(0x3B);
  Wire2.endTransmission();
  Wire2.requestFrom(gyro_address, 14);

  //zczytanie wartosci z mpu do acc i gyro
  acc_axis[1] = Wire2.read() << 8 | Wire2.read();
  acc_axis[2] = Wire2.read() << 8 | Wire2.read();
  acc_axis[3] = Wire2.read() << 8 | Wire2.read();
  temperature = Wire2.read() << 8 | Wire2.read();
  gyro_axis[1] = Wire2.read() << 8 | Wire2.read();
  gyro_axis[2] = Wire2.read() << 8 | Wire2.read();
  gyro_axis[3] = Wire2.read() << 8 | Wire2.read();
  gyro_axis[2] *= -1;
  gyro_axis[3] *= -1;

  //standaryzacja odczytanych wartosci
  acc_axis[1] -= manual_acc_pitch_cal_value;
  acc_axis[2] -= manual_acc_roll_cal_value;
  gyro_axis[1] -= manual_gyro_roll_cal_value;
  gyro_axis[2] -= manual_gyro_pitch_cal_value;
  gyro_axis[3] -= manual_gyro_yaw_cal_value;
}
