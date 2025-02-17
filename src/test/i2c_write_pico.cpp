#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 0
#define SCL_PIN 1
#define DATA_LEN 32
#define ADDR 0x08

byte buffer[DATA_LEN];

void send(){
    Wire.write(buffer, DATA_LEN);
}

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.setClock(400000);
    Wire.begin(ADDR);
    Serial.print("finished Wire setup");

    for (int i=0; i<DATA_LEN; i++){
        buffer[i] = i+1;
    }
}

uint8_t counter = 0;

void loop(){
    Serial.print("running");
    Wire.onRequest(send);
    counter++;
    delay(10);
}
