#include <Arduino.h>
#include <SoftwareSerial.h>

#define TX_PIN 2
#define RX_PIN 3
#define SERIAL_SIZE 128
#define DATA_LEN 30

byte buffer[DATA_LEN];
SoftwareSerial swSerial(RX_PIN, TX_PIN);

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    swSerial.begin(115200);
}

void loop(){
    if(swSerial.available()>=DATA_LEN){
        while(swSerial.peek()!=1) {
            Serial.println("first byte not 1");
            swSerial.read();
        }
        float startTime = micros();
        int len = swSerial.readBytes(buffer, DATA_LEN);
        float endTime = micros();
        Serial.printf("Time: %f \n", endTime - startTime);
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
