// Loads 0xB2 into a 74HC165 (a /PL pulse on D8) and reads only the first
// FOUR bits of the stream from Q7 (D10), MSB first, each sampled before the
// CP (D9) rise that shifts it on, exactly as in shiftin_165.ino. The four
// bits are driven out to the 74HC595 (SER D11, SRCLK D12, RCLK D13): the
// top nibble of 0xB2 is 0b1011, so the 595 shows 0x0B. This is the
// mid-stream case: the program leaves the register half-read.
const int plPin = 8;
const int cpPin = 9;
const int dataPin = 10;
const int serPin = 11;
const int srclkPin = 12;
const int rclkPin = 13;

void setup() {
  pinMode(plPin, OUTPUT);
  pinMode(cpPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(serPin, OUTPUT);
  pinMode(srclkPin, OUTPUT);
  pinMode(rclkPin, OUTPUT);
  digitalWrite(plPin, HIGH);
  digitalWrite(cpPin, LOW);
  digitalWrite(serPin, LOW);
  digitalWrite(srclkPin, LOW);
  digitalWrite(rclkPin, LOW);

  digitalWrite(plPin, LOW);
  digitalWrite(plPin, HIGH);

  uint8_t value = 0;
  for (uint8_t i = 0; i < 4; i++) {
    value = (value << 1) | digitalRead(dataPin);
    digitalWrite(cpPin, HIGH);
    digitalWrite(cpPin, LOW);
  }
  shiftOut(serPin, srclkPin, MSBFIRST, value);
  digitalWrite(rclkPin, HIGH);
  digitalWrite(rclkPin, LOW);
}

void loop() {
}
