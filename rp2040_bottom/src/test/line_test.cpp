#include <Arduino.h>
#include <Line.h>
#include <Adafruit_NeoPixel.h>

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define S0_PIN 0
#define S1_PIN 1
#define S2_PIN 2
#define INPUT_PIN 26
Line lineMux(S0_PIN, S1_PIN, S2_PIN, INPUT_PIN);

void setup(){
    Serial.begin(115200);

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    for (byte pin=0; pin<8; pin++){
        int value = lineMux.readRawValue(pin);
        Serial.print(String(value) + " ");
    }
    Serial.println();
}
