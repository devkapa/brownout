#include <DHT.h>

DHT dht(2, DHT11);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  delay(2100);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("read failed"));
    return;
  }
  Serial.print(F("H="));
  Serial.print(h, 1);
  Serial.print(F(" T="));
  Serial.println(t, 1);
}
