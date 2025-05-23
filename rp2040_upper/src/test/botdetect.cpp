#include <Arduino.h>
#include <VL53L5CX.h>
#include <Adafruit_NeoPixel.h>

#define SCL_PIN 7
#define SDA_PIN 6

#define ADDR2 0x31           // Only LiDAR 2 used
#define LPIN2 2           // XSHUT pin for LiDAR 2

#define SENSOR_WIDTH 8
#define SENSOR_FREQ 10

#define PICO_LED_PIN 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED_PIN, NEO_GRB + NEO_KHZ800);

#define STRIP_LED_PIN 26
#define STRIP_LED_COUNT 6
#define STRIP_LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(STRIP_LED_COUNT, STRIP_LED_PIN, NEO_GRB + NEO_KHZ800);

// Only one sensor
VL53L5CX vlLidar2(SCL_PIN, SDA_PIN, LPIN2, ADDR2, SENSOR_WIDTH, SENSOR_FREQ, Wire1);

// Utility: Set LED range to color
void setLED(int first, int last, uint32_t color){
    for (int i=first; i<=last; i++){
        strip.setPixelColor(i, color);
    }
    strip.show();
}

// Utility: Print 8x8 readings, inverted columns for visual match
void printReadings(int16_t arr[]){
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
    setLED(0, STRIP_LED_COUNT-1, strip.Color(0, 0, 15));  // Dim blue at start

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(0, 0, 15));
    pico_led.show();

    pinMode(LPIN2, OUTPUT);
    digitalWrite(LPIN2, LOW);   // Ensure sensor held low at boot

    vlLidar2.initWire();

    pico_led.setPixelColor(0, pico_led.Color(0, 15, 0));  // Green: Wire ready
    pico_led.show();

    vlLidar2.init();

    pico_led.setPixelColor(0, pico_led.Color(15, 0, 0));  // Red: Sensor ready
    pico_led.show();
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));  // Yellow heartbeat
    pico_led.show();

    bool status2 = vlLidar2.updateData();
    if(status2) {
        setLED(0, 0, strip.Color(0, 15, 0));   // Green = data OK
        Serial.println("LiDAR 2:");
        printReadings(vlLidar2.data.distance_mm);
    }
    else {
        setLED(0, 0, strip.Color(15, 0, 0));   // Red = miss
    }
}
