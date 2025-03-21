#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Line.h>

#define DEBUGGING

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define SDA_PIN 6
#define SCL_PIN 7
#define I2C_DATA_LEN 16
#define ADDR 0x08
byte sendBuffer[I2C_DATA_LEN];
// 0: start byte = 1
// 1-5: top cam
// 6-10: ballcap lidar
// 11: tofsense lidar gate
// 12-15: line sensors

Line lineMux1(0, 1, 2, 26);
Line lineMux2(0, 1, 2, 27);
Line lineMux3(0, 1, 2, 28);
Line lineMux4(0, 1, 2, 29);

void getAllLineData(){
    sendBuffer[12] = lineMux1.readData();
    sendBuffer[13] = lineMux2.readData();
    sendBuffer[14] = lineMux3.readData();
    sendBuffer[15] = lineMux4.readData();
}

void send(){
    #ifdef DEBUGGING
    for (auto i : sendBuffer){
        Serial.print(String(i) + " ");
    }
    Serial.println();
    #endif

    Wire1.write(sendBuffer, I2C_DATA_LEN);
}

void setup(){
    Serial.begin(115200);

    Wire1.setSDA(SDA_PIN);
    Wire1.setSCL(SCL_PIN);
    Wire1.setClock(400000);
    Wire1.begin(ADDR);

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(0, 15, 15));
    strip.show();
    getAllLineData();
    
    Wire1.onRequest(send);
}
