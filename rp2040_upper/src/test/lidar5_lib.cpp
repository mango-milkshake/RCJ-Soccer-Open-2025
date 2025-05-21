#include <Arduino.h>
#include <VL53L5CX.h>
#include <Adafruit_NeoPixel.h>

#define SCL_PIN 7
#define SDA_PIN 6

#define ADDR1 0x30
#define ADDR2 0x31

#define LPIN1 2
#define LPIN2 14

#define SENSOR_WIDTH 8
#define SENSOR_FREQ 15

#define PICO_LED_PIN 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED_PIN, NEO_GRB + NEO_KHZ800);

#define STRIP_LED_PIN 26
#define STRIP_LED_COUNT 6
#define STRIP_LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(STRIP_LED_COUNT, STRIP_LED_PIN, NEO_GRB + NEO_KHZ800);

VL53L5CX vlLidar1(SCL_PIN, SDA_PIN, LPIN1, ADDR1, SENSOR_WIDTH, SENSOR_FREQ, Wire1);
VL53L5CX vlLidar2(SCL_PIN, SDA_PIN, LPIN2, ADDR2, SENSOR_WIDTH, SENSOR_FREQ, Wire1);

int lastReadTime = 0;

void setLED(int first, int last, uint32_t color){
    for (int i=first; i<=last; i++){
        strip.setPixelColor(i, color);
    }
    strip.show();
}

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
    strip.setBrightness(STRIP_LED_BRIGHTNESS);
    setLED(0, STRIP_LED_COUNT-1, strip.Color(0, 0, 15));

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(0, 0, 15));
    pico_led.show();

    pinMode(LPIN1, OUTPUT);
    pinMode(LPIN2, OUTPUT);
    digitalWrite(LPIN1, LOW);
    digitalWrite(LPIN2, LOW);

    vlLidar1.initWire();

    pico_led.setPixelColor(0, pico_led.Color(0, 15, 0));
    pico_led.show();

    vlLidar1.init();
    vlLidar2.init();

    pico_led.setPixelColor(0, pico_led.Color(15, 0, 0));
    pico_led.show();
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));
    pico_led.show();

    bool status1 = vlLidar1.updateData();
    if(status1){
        // updated new data
        // int curTime = millis();
        // Serial.printf("Time: %d \n", curTime - lastReadTime);
        // lastReadTime = curTime;
        setLED(0, 0, strip.Color(0, 15, 0));
        Serial.println("1:");
        printReadings(vlLidar1.data.distance_mm);
    }
    else setLED(0, 0, strip.Color(15, 0, 0));
    bool status2 = vlLidar2.updateData();
    if(status2) {
        setLED(4, 4, strip.Color(0, 15, 0));
        Serial.println("2:");
        printReadings(vlLidar2.data.distance_mm);
    }
    else setLED(4, 4, strip.Color(15, 0, 0));
}
