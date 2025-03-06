#include <Arduino.h>
#include <Kicker.h>
#define KICKER_PIN 3

Kicker kicker(KICKER_PIN);

void setup() {
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");
}

void loop() {
    Serial.println("running");
    kicker.kick();
}