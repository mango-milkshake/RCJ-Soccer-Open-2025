#include <Arduino.h>
#include <esp_task_wdt.h>

void setup(){
    Serial.begin(115200);
    esp_task_wdt_init(1, true); // timeout in seconds
    enableLoopWDT();
}

void loop(){
    Serial.println("hi");
    delay(2000);
}