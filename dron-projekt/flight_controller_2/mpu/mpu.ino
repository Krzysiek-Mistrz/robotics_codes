#include <Wire.h>

TwoWire Wire2(2, I2C_FAST_MODE); // Inicjalizacja I2C2 (piny B10 - SCL, B11 - SDA)

void setup() {
  Serial.begin(115200);
  Wire2.begin(); // Start I2C2
  delay(100); // Czekaj, aby upewnić się, że komunikacja I2C jest gotowa
  i2c_scanner();
}

void loop() {
  i2c_scanner();
}

void i2c_scanner(void) {
  // Zmienna do przechowywania informacji o błędzie
  uint8_t error, address;
  uint16_t nDevices;

  Serial.println("Scanning address 1 to 127...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire2.beginTransmission(address);
    error = Wire2.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(2000);
}
