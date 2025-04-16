#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 18
#define LED_COUNT 12
#define LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define ESP_LED 48
#define ESP_BRIGHTNESS 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);

void setup(){
    Serial.begin(115200);
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();
}

void loop(){
    Serial.println("running");
    esp_led.setPixelColor(0, esp_led.Color(0, 15, 0));
    esp_led.show();
    for (int i=0; i<LED_COUNT; i++){
        strip.setPixelColor(i, strip.Color(15, 15, 15));
    }
    strip.show();
}
