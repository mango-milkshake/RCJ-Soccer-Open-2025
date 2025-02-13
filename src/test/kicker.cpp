#include <Arduino.h>
#define KICKER_PIN 3

void setup() {
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");
    pinMode(KICKER_PIN, OUTPUT);
}

void loop() {
    Serial.println("running");
    digitalWrite(KICKER_PIN, HIGH);
    delay(60);
    digitalWrite(KICKER_PIN, LOW);
    delay(2000);
}