#include <Arduino.h>
#include <PID.h>
#include <IMU.h>

#define CS_PIN 5
#define MISO_PIN 4 // RX
#define MOSI_PIN 3 // TX
#define SCK_PIN 2

#define TX_PIN 0
#define RX_PIN 1

IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);
PID pid(0.01, 0, 0.02, 5000);

int counter = 0;
bool tared = false;
float startTime = 0, curTime = 0;

void setup(){
    Serial.begin(115200);
    imu.init();
    startTime = millis();
    Serial1.setRX(RX_PIN);
    Serial1.setTX(TX_PIN);
    Serial1.begin(115200);
    Serial.println("started");
}

void loop(){
    // Serial.println("running");
    curTime = millis();
    if(!tared && curTime - startTime >= 5000){
        // wait 5 seconds for imu values to stabilise
        imu.tareYaw();
        if(!tared) Serial.println("Tared IMU");
        tared = true;
    }
    if(tared){
        double curYaw = imu.readYaw();
        float rotate = constrain(pid.compute(0, curYaw), -1, 1);
        int rotate_uart = rotate*128;
        Serial.println(rotate);
        Serial1.write(1);
        Serial1.write(rotate_uart & 0xFF);
        Serial1.write((rotate_uart >> 8) & 0xFF);
    }
}
