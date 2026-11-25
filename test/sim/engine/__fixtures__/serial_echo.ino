// Echoes every received byte back upper-cased.
void setup() { Serial.begin(115200); }
void loop() { if (Serial.available()) { int c = Serial.read(); Serial.write(toupper(c)); } }
