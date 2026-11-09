// Loads the levels on a 74HC165's D0-D7 with a /PL pulse on D8, then reads
// Q7 (D10) twice through the register, MSB first. The 74HC165 shifts on the
// rising edge of CP, so each bit is sampled BEFORE the CP (D9) rise that
// moves it on — Q7 holds the bit until its own clock. The first pass reads
// the loaded byte; the second reads what the register shifted in from DS
// meanwhile (zeros with DS grounded, the same byte again with DS on Q7).
// The two bytes are ORed and shifted out to a 74HC595 (SER D11, SRCLK D12,
// RCLK D13): with D0-D7 wired to 0xB2 the 595 shows 0xB2 either way.
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

  uint8_t first = readByte();
  uint8_t second = readByte();
  shiftOut(serPin, srclkPin, MSBFIRST, first | second);
  digitalWrite(rclkPin, HIGH);
  digitalWrite(rclkPin, LOW);
}

void loop() {
}
