#include <Arduino.h>

#define TX_PIN 2
#define RX_PIN 3
#define SERIAL_SIZE 128
#define DATA_LEN 99

byte buffer[DATA_LEN];

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial1.setRxBufferSize(SERIAL_SIZE);
    Serial1.setTxBufferSize(SERIAL_SIZE);
}

void loop(){
    if(Serial1.available()>=DATA_LEN){
        while(Serial1.peek()!=1) {
            Serial.println("first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(buffer, DATA_LEN);
        if(len!=DATA_LEN || buffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : buffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            for (auto i : buffer){
                Serial.print(i);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    else{
        Serial.println("No data received");
    }
}
