#include <Arduino.h>
#include <SoftwareSerial.h>

#define TX_PIN 8
#define RX_PIN 9
#define SERIAL_SIZE 128
#define DATA_LEN 30

byte buffer[DATA_LEN];
SoftwareSerial swSerial(RX_PIN, TX_PIN);
int counter = 0, total = 0;

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    swSerial.begin(38400);
}

void loop(){
    // float loopStartTime = micros();
    if(swSerial.available()>=DATA_LEN){
        while(swSerial.available()>=DATA_LEN && swSerial.peek()!=1) {
            Serial.println("first byte not 1");
            swSerial.read();
        }
        // if(swSerial.available()>=DATA_LEN && swSerial.peek()==1){
        // float startTime = micros();
        int len = swSerial.readBytes(buffer, DATA_LEN);
        // float endTime = micros();
        // Serial.printf("Time: %f \n", endTime - startTime);
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
            total++;
            int sum = 0;
            for (auto i : buffer){
                sum += i;
                Serial.print(i);
                Serial.print(" ");
            }
            if(sum!=465) counter++;
        }
        Serial.println();
        Serial.printf("Errors: %d out of %d\n", counter, total);
        Serial.printf("Error rate: %f percent\n", (float)counter*100/total);
        // }
    }
    // else{
    //     Serial.println("No data received");
    // }
    // float loopEndTime = micros();
    // Serial.printf("Loop Time: %f \n", loopEndTime - loopStartTime);
}
