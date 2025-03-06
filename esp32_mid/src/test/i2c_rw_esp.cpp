#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define RCV_DATA_LEN 32
#define SEND_DATA_LEN 8
#define RCV_PICO_ADDR 0x08
#define SEND_PICO_ADDR 0x09

byte rcvBuffer[RCV_DATA_LEN+1], sendBuffer[SEND_DATA_LEN];
SemaphoreHandle_t i2cMutex;

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
    for (int i=0; i<SEND_DATA_LEN; i++) sendBuffer[i] = (i+1)*30;
    while(1){
        if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
            Wire.beginTransmission(SEND_PICO_ADDR);
            Wire.write(sendBuffer, SEND_DATA_LEN);
            Wire.endTransmission();
            xSemaphoreGive(i2cMutex);
            Serial.println("Data sent");
        }
        delay(10);
    }
}

void core1Task(void *pvParameters){
    while(1){
        if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
            byte num_bytes = Wire.requestFrom(RCV_PICO_ADDR, RCV_DATA_LEN);
            if(num_bytes != RCV_DATA_LEN){
                Serial.print("Received bad data: ");
            }
            else Serial.print("Received: ");
            for (int i=0; i<RCV_DATA_LEN; i++) {
                if (Wire.available()) {
                    rcvBuffer[i] = Wire.read();
                }
            }
            rcvBuffer[RCV_DATA_LEN] = '\0';
            xSemaphoreGive(i2cMutex);
            for (auto i : rcvBuffer){
                Serial.print(i);
                Serial.print(" ");
            }
            Serial.println();
        }
        delay(10);
    }
}

// core 1 handles reading data from cameras and top and bottom plates, and sensor fusion
void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");

    i2cMutex = xSemaphoreCreateMutex(); 
    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    Serial.println("finished Wire setup");

    xTaskCreatePinnedToCore(core0Task, "Read Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Send Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    // Serial.println("running");
}
