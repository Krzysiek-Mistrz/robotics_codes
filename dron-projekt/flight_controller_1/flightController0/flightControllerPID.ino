#include <Wire.h>
#include <Servo.h>

// Definiowanie pinow dla silnikow
const int motorPin1 = 3;
const int motorPin2 = 9;
const int motorPin3 = 10;
const int motorPin4 = 11;

// Definiowanie pinow dla MPU6050
const int MPU6050_SCL = A5;
const int MPU6050_SDA = A4;

// Obiekty dla obslugi silnikow
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

// Parametry PID
double Kp = 2.0;  // Wzmocnienie proporcjonalne
double Ki = 0.1;  // Wzmocnienie calkujace
double Kd = 1.0;  // Wzmocnienie rozniczkujace

// Zmienne PID
double pidError = 0;
double previousError = 0;
double integral = 0;

// Czas trwania lotu (w milisekundach)
const unsigned long flightDuration = 10000;

// Czas próbkowania PID (w milisekundach)
const int pidSampleTime = 20;

// Funkcja do ustawiania drona
void setupDrone() {
  // Inicjalizacja komunikacji z MPU6050
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Inicjalizacja silnikow
  motor1.attach(motorPin1);
  motor2.attach(motorPin2);
  motor3.attach(motorPin3);
  motor4.attach(motorPin4);

  // Wylaczenie silnikow na poczatku
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  delay(1000);
}

// Funkcja do kontrolowania drona
void controlDrone() {
  unsigned long startTime = millis();
  unsigned long previousMillis = 0;

  while (millis() - startTime < flightDuration) {
    unsigned long currentMillis = millis();

    // Odczyt danych z MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();

    // Algorytm PID
    pidError = atan2(AcY, AcX) * (180.0 / M_PI);  // Kat nachylenia drona w stopniach
    integral += pidError * (currentMillis - previousMillis) / 1000.0;
    double derivative = (pidError - previousError) / (currentMillis - previousMillis) * 1000.0;

    // Sygnal sterujacy PID
    double pidOutput = Kp * pidError + Ki * integral + Kd * derivative;

    // Sterowanie silnikami na podstawie PID
    motor1.writeMicroseconds(1150 + pidOutput);
    motor2.writeMicroseconds(1150 + pidOutput);
    motor3.writeMicroseconds(1150 - pidOutput);
    motor4.writeMicroseconds(1150 - pidOutput);

    // Aktualizacja zmiennych PID
    previousError = pidError;
    previousMillis = currentMillis;

    // Czas probkowania PID
    delay(pidSampleTime);
  }

  unsigned long landingStartTime = millis();
  const unsigned long landingDuration = 3000;  // Czas lądowania (w milisekundach)

  while (millis() - landingStartTime < landingDuration) {
    // Stopniowe zmniejszanie predkosci silnikow
    int throttle = 1150 - ((millis() - landingStartTime) * 1000 / landingDuration);
    
    // Ustawienie predkosci silnikow
    motor1.writeMicroseconds(throttle);
    motor2.writeMicroseconds(throttle);
    motor3.writeMicroseconds(throttle);
    motor4.writeMicroseconds(throttle);

    delay(pidSampleTime);
  }

  // Wylaczenie silnikow po zakonczeniu lotu
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}

void loop() {
  // Inicjalizacja drona
  setupDrone();

  // Rozpoczecie lotu
  controlDrone();

  /*
  while (1) {
    // Dodatkowe operacje
  }
  */
}