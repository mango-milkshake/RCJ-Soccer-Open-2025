#include <Arduino.h>
#include <Dribbler.h>
#include <Motor.h>

// Dribbler
#define MOSI_PIN 12
#define MISO_PIN 13
#define SCK_PIN 14
#define CS_PIN 15
#define DRIBBLER_IN1 34
#define DRIBBLER_IN2 33
#define DRIBBLER_NFAULT 35
#define NSLEEP_PIN 36
#define DRVOFF_PIN 37
MotorDriver dribblerMD(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

uint8_t dribbler_maxspeed = 60;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
float lastFault = 0;

void checkFault(){
    bool faulted = false;
    if(digitalRead(dribbler.nfault)==LOW) faulted = true;
    if(faulted){
        Serial.println("faulted");
        float curTime = millis();
        if(curTime - lastFault >= 500){
            dribblerMD.clearFault();
            dribblerMD.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
}

void setup(){
    Serial.begin(115200);
    while(!Serial.available());
    while(Serial.available()) Serial.read();

    dribblerMD.init();
    dribblerMD.setMode();

    while(!Serial.available());
    while(Serial.available()) Serial.read();
}

void loop(){
    checkFault();
    dribbler.setSpeed(1.0);
}