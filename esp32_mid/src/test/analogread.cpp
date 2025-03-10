#include <Arduino.h>
#define VOLTAGE_PIN 2

void setup() {
    Serial.begin(115200);
    Serial.println("started");
    pinMode(VOLTAGE_PIN, INPUT);
}

void loop() {
    // Serial.println("running");
    float value = analogRead(VOLTAGE_PIN);
    value = value * 13.3 / 3.3;
    Serial.print(value);
    Serial.print("\t");
    delay(10);
}