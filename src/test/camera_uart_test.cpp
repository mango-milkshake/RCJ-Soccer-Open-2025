#include <Arduino.h>

#define TX_PIN 2
#define RX_PIN 3

byte buffer[9];

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    // Serial1.setRX(RX_PIN);
    // Serial1.setTX(TX_PIN);
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop(){
    if(Serial1.available()>=9){
        while(Serial1.peek()!=1) {
            Serial.println("first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(buffer, 9);
        if(len!=9 || buffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : buffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            float ball_angle = (float)(buffer[1] + (buffer[2]<<8)) / 128;
            float ball_dist = (float)(buffer[3] + (buffer[4]<<8)) / 128;
            float goal_angle = (float)buffer[5] + (buffer[6]<<8) / 128;
            float goal_dist = (float)buffer[7] + (buffer[8]<<8) / 128;
            Serial.print(ball_angle, 3);
            Serial.print("\t");
            Serial.print(ball_dist, 3);
            Serial.print("\t");
            Serial.print(goal_angle, 3);
            Serial.print("\t");
            Serial.print(goal_dist, 3);
        }
        Serial.println();
    }
    // else{
    //     Serial.println("No data received");
    // }
}
