#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_RCV_DATA_LEN 32
#define I2C_SEND_DATA_LEN 8
#define I2C_RCV_PICO_ADDR 0x08
#define I2C_SEND_PICO_ADDR 0x09

#define PICO_TX_PIN 16
#define PICO_RX_PIN 17
#define SERIAL_DATA_LEN 7

HardwareSerial Serial0(0);
HardwareSerial Serial2(1);
HardwareSerial Serial2(2);

byte uartBuffer[SERIAL_DATA_LEN];

byte rcvBuffer[I2C_RCV_DATA_LEN+1], sendBuffer[I2C_SEND_DATA_LEN];
SemaphoreHandle_t i2cMutex, coordMutex;

#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21
float cur_x, cur_y, cur_heading;
float target_x = FIELD_WIDTH/2, target_y = FIELD_HEIGHT/2;

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
    for (int i=0; i<I2C_SEND_DATA_LEN; i++) sendBuffer[i] = (i+1)*30;
    float self_x = 0, self_y = 0, self_heading = 0;
    while(1){
        if(xSemaphoreTake(coordMutex, 0)){
            self_x = cur_x;
            self_y = cur_y;
            self_heading = cur_heading;
            xSemaphoreGive(coordMutex);
        }
        if(xSemaphoreTake(i2cMutex, 0)){
            Wire.beginTransmission(I2C_SEND_PICO_ADDR);
            Wire.write(sendBuffer, I2C_SEND_DATA_LEN);
            Wire.endTransmission();
            xSemaphoreGive(i2cMutex);
            Serial.println("Data sent");
        }
        delay(10);
    }
}

// core 1 handles receiving data and processing to get final self and ball coordinates
void core1Task(void *pvParameters){
    while(1){
        if(Serial2.available()>=SERIAL_DATA_LEN){
            while(Serial2.peek()!=1) {
                Serial.println("first byte not 1");
                Serial2.read();
            }
            int len = Serial2.readBytes(uartBuffer, SERIAL_DATA_LEN);
            if(len!=SERIAL_DATA_LEN || uartBuffer[0]!=1){
                Serial.print("Received bad data: length: ");
                Serial.print(len);
                Serial.print(", data: ");
                for (auto i : uartBuffer) {
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            else{
                float coord_x = (float)(uartBuffer[1] + (uartBuffer[2]<<8)) / 128;
                float coord_y = (float)(uartBuffer[3] + (uartBuffer[4]<<8)) / 128;
                float heading = (float)uartBuffer[5] + (uartBuffer[6]<<8) / 128;
                Serial.print(coord_x, 3);
                Serial.print("\t");
                Serial.print(coord_y, 3);
                Serial.print("\t");
                Serial.print(heading, 3);
                if(xSemaphoreTake(coordMutex, portMAX_DELAY)){
                    cur_x = coord_x;
                    cur_y = coord_y;
                    cur_heading = heading;
                    xSemaphoreGive(coordMutex);
                    Serial.println("Coordinate data updated");
                }
                // for (auto i : uartBuffer){
                //     Serial.print(i);
                //     Serial.print(" ");
                // }
            }
            Serial.println();
        }
        else{
            Serial.println("No data received");
        }
    }
}

// core 1 handles reading data from cameras and top and bottom plates, and sensor fusion
void setup(){
    Serial.begin(115200);

    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    // Serial.println("started");

    Serial2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    i2cMutex = xSemaphoreCreateMutex(); 
    coordMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    Serial.println("finished Wire setup");

    xTaskCreatePinnedToCore(core0Task, "Read Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Send Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    // Serial.println("running");
}
