#include <Arduino.h>
#include <Kicker.h>
#include <Adafruit_NeoPixel.h>

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define KICKER_PIN 3
Kicker kicker(KICKER_PIN);

int delayTime = 1000;
int iterations = 5;

void setup() {
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 15, 15));
    strip.show();
    Serial.println("started");
}

void loop() {
    Serial.println("running");
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
    // kicker.kick();

    for (int i=delayTime; i>=50; i-=50){
        for (int j=0; j<iterations; j++){
            digitalWrite(KICKER_PIN, HIGH);
            delayMicroseconds(i);
            digitalWrite(KICKER_PIN, LOW);
            delayMicroseconds(i);
        }
    }
}