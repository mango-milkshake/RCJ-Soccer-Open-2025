#include <Arduino.h>
#include <Wire.h>

#define SDA0_PIN 4
#define SCL0_PIN 5
#define SDA1_PIN 8
#define SCL1_PIN 9
#define SEND_DATA_LEN 30
#define RCV_DATA_LEN 30
#define ADDR 0x08

byte sendBuffer[SEND_DATA_LEN], rcvBuffer[RCV_DATA_LEN];

void send(){
    Wire1.beginTransmission(ADDR);
    Wire1.write(sendBuffer, SEND_DATA_LEN);
    Wire1.endTransmission();
    Serial.println("Sent data");
}

void receive(){
    byte num_bytes = Wire.requestFrom(ADDR, RCV_DATA_LEN);
    if(num_bytes != RCV_DATA_LEN){
        Serial.print("Received bad data: ");
        return;
    }
    else Serial.print("Received: ");
    for (int i=0; i<RCV_DATA_LEN; i++) {
        if (Wire.available()) {
            rcvBuffer[i] = Wire.read();
        }
    }
    rcvBuffer[RCV_DATA_LEN] = '\0';
    for (auto i : rcvBuffer){
        Serial.printf("%d ", i);
    }
    Serial.println();
}


void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    Wire.begin(SDA0_PIN, SCL0_PIN, 400000);
    Wire1.begin(SDA1_PIN, SCL1_PIN, 400000);
    Serial.println("finished Wire setup");

    for (int i=0; i<SEND_DATA_LEN; i++){
        if(i==0) sendBuffer[i] = 1;
        else sendBuffer[i] = i+31;
    }
}

void loop(){
    Serial.print("running");
    receive();
    send();
    // delay(3);
}
