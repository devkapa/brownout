// Loads the levels on a 74HC165's D0-D7 with a /PL pulse on D8, then reads
// Q7 (D10) TWICE through the register, MSB first, each bit sampled before
// the CP (D9) rise that shifts it on (see shiftin_165.ino). Only the SECOND
// pass is driven out to the 74HC595 (SER D11, SRCLK D12, RCLK D13): with DS
// grounded the register has emptied, so the 595 shows 0x00, and with DS on
// Q7 the register rotates, so the second pass reads the same byte again
// (0xB2 with D0-D7 wired to it).
const int plPin = 8;
const int cpPin = 9;
const int dataPin = 10;
const int serPin = 11;
const int srclkPin = 12;
const int rclkPin = 13;

uint8_t readByte() {
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    value = (value << 1) | digitalRead(dataPin);
    digitalWrite(cpPin, HIGH);
    digitalWrite(cpPin, LOW);
  }
  return value;
}

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

  readByte();
  uint8_t second = readByte();
  shiftOut(serPin, srclkPin, MSBFIRST, second);
  digitalWrite(rclkPin, HIGH);
  digitalWrite(rclkPin, LOW);
}

void loop() {
}
