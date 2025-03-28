#include <Arduino.h>
#include <SPI.h>

#define MOSI_PIN 7
#define MISO_PIN 6
#define SCK_PIN 5
#define CS_PIN 4

#define speedMaximum 100000
SPISettings CamSetting(speedMaximum, MSBFIRST, SPI_MODE0);

SPIClass vspi(SPI);

#define DATA_LEN 9
byte buffer[DATA_LEN];

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    vspi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
}

void loop(){

    vspi.beginTransaction(CamSetting);
    digitalWrite(CS_PIN, LOW);
    vspi.transferBytes(NULL, buffer, DATA_LEN);
    digitalWrite(CS_PIN, HIGH);
    vspi.endTransaction();

    for (auto i : buffer){
        Serial.print(i);
        Serial.print(" ");
    }

    float ball_angle = (float)(buffer[1] + (buffer[2]<<8)) / 128;
    float ball_dist = (float)(buffer[3] + (buffer[4]<<8)) / 128;
    float goal_angle = (float)buffer[5] + (buffer[6]<<8) / 128;
    float goal_dist = (float)buffer[7] + (buffer[8]<<8) / 128;
    // Serial.print(ball_angle, 3);
    // Serial.print("\t");
    // Serial.print(ball_dist, 3);
    // Serial.print("\t");
    // Serial.print(goal_angle, 3);
    // Serial.print("\t");
    // Serial.print(goal_dist, 3);

    Serial.println();
    delay(10);
}