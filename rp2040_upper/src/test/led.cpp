#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

void setup()
{
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(brightness);
  strip.setPixelColor(0, strip.Color(15, 15, 0));
  strip.show();
  delay(1000);
}

void loop()
{
  Serial.println("looping");
  strip.setPixelColor(0, strip.Color(15, 0, 15));
  strip.show();

}