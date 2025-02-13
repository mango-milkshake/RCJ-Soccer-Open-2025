#include <Lidar.h>
#include <chrono>
#include <vector>

#define NUM_LIDARS 8
#define NUM_EACH_BUS 8
#define DIST_FROM_CENTRE 8.5
float distRaw[NUM_LIDARS];
std::vector<Lidar> lidar;
uint8_t scl[2] = {1, 5}, sda[2] = {0, 4};
float calib[NUM_LIDARS] = {0, 0, 0, 0, 0, 0, 0, 0};
double fps = 0;

void setup(){
    Serial.begin(115200);

    for (uint8_t i=0; i<NUM_LIDARS; i++){
        uint8_t bus;
        if(i<NUM_EACH_BUS) bus = 0;
        else bus = 1;
        lidar.emplace_back(scl[bus], sda[bus], i+1, calib[i]+DIST_FROM_CENTRE);
    }

    for (int i=0; i<2; i++){
        Analog_IIC_Init(scl[i], sda[i]);
    }
}

void loop(){
    for (int i=0; i<NUM_LIDARS; i++){
        auto start_time = std::chrono::steady_clock::now();
        distRaw[i] = lidar[i].readRaw();
        auto cur_time = std::chrono::steady_clock::now();
        std::chrono::nanoseconds diff = cur_time - start_time;
        fps = 0.9 * fps + 0.1 * (1000000000 / diff.count());

        // Serial.print("Read Sensor ");
        Serial.print(i+1);
        // Serial.print(" - time: ");
        // Serial.println(fps);

        // Serial.print(" dist: ");
        Serial.print(" ");
        Serial.print(distRaw[i]);
        Serial.print("\t");
        // Serial.println();
    }
    Serial.println();
    // delay(100);
}
