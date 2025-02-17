#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 0
#define SCL_PIN 1
#define DATA_LEN 32
#define ADDR 0x08

byte buffer[DATA_LEN+1];

void receive(int num_bytes){
    if(num_bytes != DATA_LEN){
        Serial.print("Received bad data: ");
    }
    else Serial.print("Received: ");
    for (int i=0; i<DATA_LEN; i++){
        if(Wire.available()){
            buffer[i] = Wire.read();
        }
    }
    buffer[DATA_LEN] = '\0';
    for (auto i : buffer){
        Serial.print(i);
        Serial.print(" ");
    }
    Serial.println();
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
    Serial.println("finished Wire setup");
}

void loop(){
    // Serial.println("running");
    Wire.onReceive(receive);
    delay(100);
}
