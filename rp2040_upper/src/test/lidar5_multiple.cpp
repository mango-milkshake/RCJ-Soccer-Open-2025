#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

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

SparkFun_VL53L5CX sensor1;
VL53L5CX_ResultsData measurementData1; // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX sensor2;
VL53L5CX_ResultsData measurementData2;

#define DELAY_TIME 0

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

    Wire.setSCL(SCL_PIN);
    Wire.setSDA(SDA_PIN);
    Wire.begin();
    Wire.setClock(400000);

    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);

    strip.setPixelColor(0, strip.Color(0, 15, 0));
    strip.show();

    digitalWrite(XSHUT1, HIGH);
    // delay(DELAY_TIME);
    while(sensor1.begin() == false){
        Serial.println("Sensor 1 not found");
        // while(1) Serial.println("Sensor 1 not found");
    }
    // delay(DELAY_TIME);
    strip.setPixelColor(0, strip.Color(15, 15, 15));
    strip.show();
    sensor1.setAddress(ADDR1);
    // delay(DELAY_TIME);
    // digitalWrite(XSHUT1, LOW);

    strip.setPixelColor(0, strip.Color(15, 0, 0));
    strip.show();

    digitalWrite(XSHUT2, HIGH);
    // delay(DELAY_TIME);
    while(sensor2.begin() == false){
        Serial.println("Sensor 2 not found");
        // while(1) Serial.println("Sensor 2 not found");
    }
    // delay(DELAY_TIME);
    strip.setPixelColor(0, strip.Color(15, 15, 15));
    strip.show();
    sensor2.setAddress(ADDR2);
    // delay(DELAY_TIME);
    // digitalWrite(XSHUT2, LOW);

    strip.setPixelColor(0, strip.Color(0, 15, 15));
    strip.show();

    digitalWrite(XSHUT1, HIGH);
    digitalWrite(XSHUT2, HIGH);

    sensor1.setWireMaxPacketSize(128);
    // if(sensor1.begin() == false){
    //     Serial.println("Sensor 1 not found");
    //     while(1) ;
    // }
    sensor1.setResolution(8*8);
    sensor1.setRangingFrequency(15);
    sensor1.startRanging();

    strip.setPixelColor(0, strip.Color(15, 0, 15));
    strip.show();

    sensor2.setWireMaxPacketSize(128);
    // if(sensor2.begin() == false){
    //     Serial.println("Sensor 1 not found");
    //     while(1) ;
    // }
    sensor2.setResolution(8*8);
    sensor2.setRangingFrequency(15);
    sensor2.startRanging();

    strip.setPixelColor(0, strip.Color(15, 0, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();

    if(sensor1.isDataReady()){
        if(sensor1.getRangingData(&measurementData1)){
            Serial.println("01:");
            printReadings(measurementData1.distance_mm);
        }
    }

    if(sensor2.isDataReady()){
        if(sensor2.getRangingData(&measurementData2)){
            Serial.println("02:");
            printReadings(measurementData2.distance_mm);
        }
    }
}
