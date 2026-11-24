#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;) {}
  }
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.fillRect(100, 40, 20, 16, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(4, 4);
  display.print(F("de:volt"));
  display.drawPixel(64, 32, SSD1306_WHITE);
  display.display();
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {}
