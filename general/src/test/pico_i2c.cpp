#include <Arduino.h>
#include <Wire.h>

#define SDA0_PIN 0
#define SCL0_PIN 1
#define SDA1_PIN 2
#define SCL1_PIN 3
#define SEND_DATA_LEN 30
#define RCV_DATA_LEN 30
#define ADDR 0x08

byte sendBuffer[SEND_DATA_LEN], rcvBuffer[RCV_DATA_LEN];

void send(){
    // Wire.beginTransmission(ADDR);
    Wire.write(sendBuffer, SEND_DATA_LEN);
    // Wire.endTransmission();
    Serial.println("Sent data");
}

void receive(int num_bytes){
    if(num_bytes != RCV_DATA_LEN){
        Serial.print("Received bad data length");
        return;
    }
    else Serial.print("Received: ");
    for (int i=0; i<RCV_DATA_LEN; i++){
        if(Wire1.available()){
            rcvBuffer[i] = Wire1.read();
        }
    }
    rcvBuffer[RCV_DATA_LEN] = '\0';
    for (auto i : rcvBuffer){
        Serial.print(i);
        Serial.print(" ");
    }
    Serial.println();
}


void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    Wire.setSDA(SDA0_PIN);
    Wire.setSCL(SCL0_PIN);
    Wire.begin(ADDR);
    Wire.onRequest(send);

    Wire1.setSDA(SDA1_PIN);
    Wire1.setSCL(SCL1_PIN);
    Wire1.begin(ADDR);
    Wire1.onReceive(receive);
    Serial.print("finished Wire setup");

    for (int i=0; i<SEND_DATA_LEN; i++){
        sendBuffer[i] = i+1;
    }
}

void loop(){
    // Serial.println("running");
}
