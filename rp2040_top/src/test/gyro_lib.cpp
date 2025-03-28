#include <Arduino.h>
#include <IMU.h>
#include <Adafruit_NeoPixel.h>

// #define SECOND_BUS

#ifdef SECOND_BUS
#define SPI SPI1
#define CS_PIN 13
#define MISO_PIN 12 // RX
#define MOSI_PIN 15 // TX
#define SCK_PIN 14
#else
#define CS_PIN 5
#define MISO_PIN 4 // RX
#define MOSI_PIN 3 // TX
#define SCK_PIN 2
#endif

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");
    imu.init();

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();
}

float lastTime = 0, curTime = 0;
int counter = 0;

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    if(counter==800) imu.tareAll();
    imu.updateAllData();
    double res = imu.yaw;

    // if(res==631) Serial.println("Waiting for data");
    if(res!=631) {
        curTime = millis();
        float duration = curTime - lastTime;
        Serial.print("Yaw: ");
        Serial.println(res);
        // Serial.print("duration: ");
        // Serial.println(duration);
        // Serial.println(counter);
        Serial.println();
        lastTime = curTime;
        counter++;
    }
}
