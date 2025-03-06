#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("started");
    pinMode(6, INPUT);
}

void loop() {
    Serial.println("running");
    Serial.print(analogRead(6));
    Serial.print("\t");
    delay(10);
}