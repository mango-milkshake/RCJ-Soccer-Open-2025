#include <Arduino.h>
#include <IMU.h>

// #define SECOND_BUS

#ifdef SECOND_BUS
#define SPI SPI1
#define CS_PIN 13
#define MISO_PIN 12 // RX
#define MOSI_PIN 15 // TX
#define SCK_PIN 14
#else
#define CS_PIN 5
#define MISO_PIN 4 // RX
#define MOSI_PIN 3 // TX
#define SCK_PIN 2
#endif

IMU imu(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, SPI);

void setup(){
    Serial.begin(115200);
    while(!Serial.available()) ;
    while(Serial.available()) Serial.read();
    Serial.println("started");
    imu.init();
}

int16_t accel_x, accel_y, accel_z;

void loop(){
    imu.updateAllData();
    accel_x = imu.accelX;
    accel_y = imu.accelY;
    accel_z = imu.accelZ;

    Serial.print("Accel: ");
    Serial.print(accel_x);
    Serial.print(" ");
    Serial.print(accel_y);
    Serial.print(" ");
    Serial.print(accel_z);
    Serial.print(" ");
    Serial.println();
}
