#define NUM_SENSORS 8
#define SENSOR_PINS {13, 12, 14, 27, 26, 25, 33, 32}

const uint8_t sensorPins[NUM_SENSORS] = SENSOR_PINS;
unsigned long sensorValues[NUM_SENSORS] = {0};


void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize the sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], LOW);
  }
  delay(50);
}


void loop() {
  // Step 2: Set the I/O lines to an output and drive them high
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  // Step 3: Allow at least 10 μs for the sensor outputs to rise
  delayMicroseconds(10);

  // Step 4: Make the I/O lines inputs (high impedance)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Step 5: Measure the time for the voltage to decay by waiting for the I/O lines to go low
  unsigned long startTime = micros();
  bool allSensorsLow = false;
  while (!allSensorsLow) {
    allSensorsLow = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (digitalRead(sensorPins[i]) == HIGH) {
        sensorValues[i] = micros() - startTime;
        allSensorsLow = false;
      }
    }
  }

  // Print the sensor values for debugging
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  
  delay(100);
  // Add a small delay to prevent flooding the serial output
}