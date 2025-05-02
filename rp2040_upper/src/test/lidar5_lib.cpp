#include <Arduino.h>
#include <VL53L5CX.h>
#include <Adafruit_NeoPixel.h>

#define SCL_PIN 5
#define SDA_PIN 4

#define ADDR1 0x30
#define ADDR2 0x31

#define XSHUT1 1
#define XSHUT2 2

#define SENSOR_WIDTH 8
#define SENSOR_FREQ 15

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

VL53L5CX vlLidar1(SCL_PIN, SDA_PIN, XSHUT1, ADDR1, SENSOR_WIDTH, SENSOR_FREQ, Wire);
VL53L5CX vlLidar2(SCL_PIN, SDA_PIN, XSHUT2, ADDR2, SENSOR_WIDTH, SENSOR_FREQ, Wire);

int lastReadTime = 0;

void printReadings(int16_t arr[]){
    // print readings inverted (reflects reality)
    Serial.println("==================================================================");
    for (int y = 0; y <= SENSOR_WIDTH * (SENSOR_WIDTH - 1); y += SENSOR_WIDTH){
        Serial.print("||");
        for (int x = SENSOR_WIDTH - 1; x >= 0; x--){
            if(arr[x+y] < 10) Serial.print("   ");
            else if(arr[x+y] < 1000) Serial.print("  ");
            else Serial.print(" ");
            Serial.print(arr[x+y]);
            if(arr[x+y] < 100) Serial.print("  ");
            else Serial.print(" ");
            Serial.print("||");
        }
        Serial.println();
        Serial.println("==================================================================");
    }
    Serial.println();
}

void setup(){
    Serial.begin(115200);

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(0, 0, 15));
    strip.show();

    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);

    vlLidar1.initWire();

    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();

    vlLidar1.init();
    vlLidar2.init();

    strip.setPixelColor(0, strip.Color(15, 0, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    bool status1 = vlLidar1.updateData();
    if(status1){
        // updated new data
        // int curTime = millis();
        // Serial.printf("Time: %d \n", curTime - lastReadTime);
        // lastReadTime = curTime;
        Serial.println("1:");
        printReadings(vlLidar1.data.distance_mm);
    }
    bool status2 = vlLidar2.updateData();
    if(status2) {
        Serial.println("2:");
        printReadings(vlLidar2.data.distance_mm);
    }
}
