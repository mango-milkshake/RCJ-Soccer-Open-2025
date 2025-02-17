#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define DATA_LEN 32
#define ADDR 0x08

byte buffer[DATA_LEN];

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    Serial.print("finished Wire setup");

    for (int i=0; i<DATA_LEN; i++){
        buffer[i] = i+1;
    }
}

void loop(){
    Serial.print("running");
    Wire.beginTransmission(ADDR);
    Wire.write(buffer, DATA_LEN);
    Wire.endTransmission();
    delay(10);
}
