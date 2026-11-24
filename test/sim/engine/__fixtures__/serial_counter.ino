// Prints a counter forever and toggles D13 after every line: far more than the
// 64-byte transmit buffer, so the sketch only keeps running if the UART drains it.
unsigned long n = 0;
void setup() { Serial.begin(9600); pinMode(13, OUTPUT); }
void loop() { Serial.print("count "); Serial.println(n++); digitalWrite(13, n & 1); }
