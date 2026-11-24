#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println(F("no BMP280"));
    for (;;) {}
  }
}

void loop() {
  Serial.print(F("T="));
  Serial.print(bmp.readTemperature(), 2);
  Serial.print(F(" P="));
  Serial.println(bmp.readPressure(), 1);
  delay(100);
}
