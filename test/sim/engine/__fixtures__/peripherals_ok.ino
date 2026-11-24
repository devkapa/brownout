// Uses EEPROM (twice, so the second write waits for the first), Wire and SPI, then
// reports over Serial. Each library busy-waits on its peripheral, so without one the
// sketch never reaches the report.
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
void setup() {
  Serial.begin(115200);
  EEPROM.write(0, 42);
  EEPROM.write(1, 43);
  Wire.begin();
  Wire.beginTransmission(0x3C);
  Wire.write(0);
  byte ack = Wire.endTransmission();
  SPI.begin();
  byte spi = SPI.transfer(0x5A);
  Serial.print("eeprom ");
  Serial.print(EEPROM.read(0));
  Serial.print(",");
  Serial.print(EEPROM.read(1));
  Serial.print(" i2c ");
  Serial.print(ack);
  Serial.print(" spi ");
  Serial.println(spi, HEX);
}
void loop() {}
