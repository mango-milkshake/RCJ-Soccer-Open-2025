#include <Arduino.h>
#include <VL53L5CX.h>

#define SCL_PIN 5
#define SDA_PIN 4

#define SENSOR_WIDTH 8
#define SENSOR_FREQ 15

VL53L5CX vlLidar(SCL_PIN, SDA_PIN, SENSOR_WIDTH, SENSOR_FREQ, Wire);

int lastReadTime = 0;

void setup(){
    Serial.begin(115200);
    vlLidar.init();
}

void loop(){
    bool status = vlLidar.updateData();
    if(status){
        // updated new data
        int curTime = millis();
        Serial.printf("Time: %d \n", curTime - lastReadTime);
        lastReadTime = curTime;
        // print readings inverted (reflects reality)
        for (int y = 0; y <= SENSOR_WIDTH * (SENSOR_WIDTH - 1); y += SENSOR_WIDTH){
            for (int x = SENSOR_WIDTH - 1; x >= 0; x--){
                Serial.print(vlLidar.data.distance_mm[x+y]);
                Serial.print(",");
            }
            Serial.println();
        }
        Serial.println();
    }
}
