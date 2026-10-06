// Shifts 0xB2 into a 74HC595 with shiftOut(), MSB first, then pulses the
// latch once. D11 drives SER, D12 drives SRCLK and D8 drives RCLK.
const int dataPin = 11;
const int clockPin = 12;
const int latchPin = 8;

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 0xB2);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}

void loop() {
}
