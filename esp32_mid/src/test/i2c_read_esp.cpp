#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define DATA_LEN 32
#define ADDR 0x08

byte buffer[DATA_LEN+1];

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    Serial.println("finished Wire setup");
}

void loop(){
    // Serial.println("running");
    byte num_bytes = Wire.requestFrom(ADDR, DATA_LEN);
    if(num_bytes != DATA_LEN){
        Serial.print("Received bad data: ");
    }
    else Serial.print("Received: ");
    for (int i=0; i<DATA_LEN; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
    buffer[DATA_LEN] = '\0';
    for (auto i : buffer){
        Serial.print(i);
        Serial.print(" ");
    }
    Serial.println();
    delay(100);
}
