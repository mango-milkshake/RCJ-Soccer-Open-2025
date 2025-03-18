#include <Arduino.h>
#define VOLTAGE_PIN 2

void setup() {
    Serial.begin(115200);
    Serial.println("started");
    pinMode(VOLTAGE_PIN, INPUT);
    analogSetAttenuation(ADC_11db);
}

void loop() {
    // Serial.println("running");
    float value = analogRead(VOLTAGE_PIN);
    float voltage = (value+1) * 13.3 / 4096;
    voltage = voltage / 12.3 * 11.8;
    Serial.print(value);
    Serial.print("\t");
    Serial.println(voltage);
    delay(10);
}