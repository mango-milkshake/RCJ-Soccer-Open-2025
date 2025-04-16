#include <Arduino.h>
#include <LidarGate.h>
#include <Adafruit_NeoPixel.h>

// #define PRINT_RAW_DIST
#define PRINT_STATUS

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define LIDAR_GATE_SDA_PIN 8
#define LIDAR_GATE_SCL_PIN 9
#define LIDAR_GATE_ID 0
LidarGate lidargate(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN, LIDAR_GATE_ID);

void setup(){
    Serial.begin(115200);

    Analog_IIC_Init(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN);

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    #ifdef PRINT_RAW_DIST
    float dist = lidargate.readRaw();
    Serial.println(dist);
    #endif

    #ifdef PRINT_STATUS
    bool status = lidargate.checkBallCap();
    Serial.println(status);
    #endif
}