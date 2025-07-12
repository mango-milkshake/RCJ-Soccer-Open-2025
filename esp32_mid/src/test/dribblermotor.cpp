#include <Arduino.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Adafruit_NeoPixel.h>

// Dribbler
#define MOSI_PIN 12
#define MISO_PIN 13
#define SCK_PIN 14
#define CS_PIN 15
#define DRIBBLER_IN1 47
#define DRIBBLER_IN2 48
#define DRIBBLER_NFAULT 21
#define NSLEEP_PIN 6
#define DRVOFF_PIN 7
#define IPROPI_PIN 5
MotorDriver dribblerMD(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN, IPROPI_PIN);

uint8_t dribbler_maxspeed = 150, dribbler_minspeed = 40;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
int lastFault = 0;

void checkFault(){
    bool faulted = false;
    if(digitalRead(dribbler.nfault)==LOW) faulted = true;
    if(faulted){
        Serial.println("faulted");
        int curTime = millis();
        if(curTime - lastFault >= 500){
            dribblerMD.clearFault();
            dribblerMD.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
}

void setup(){
    Serial.begin(115200);
    // delay(1000);

    // while(!Serial.available());
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    // esp_led.begin();
    // esp_led.setBrightness(ESP_BRIGHTNESS);
    // esp_led.show();

    dribblerMD.init();
    dribblerMD.setMode();
}

void loop(){
    // Serial.println("running");
    // esp_led.setPixelColor(0, esp_led.Color(0, 15, 0));
    // esp_led.show();
    checkFault();

    dribbler.setSpeed(50);
    // analogWrite(47, 127);
}