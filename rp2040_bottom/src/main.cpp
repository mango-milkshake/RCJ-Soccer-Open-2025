#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Line.h>
#include <LidarGate.h>
#include <vector>

#define DEBUGGING

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);
uint32_t colour;

#define KICKER_LOGIC_PIN 3

#define SDA_PIN 6
#define SCL_PIN 7
#define I2C_DATA_LEN 3
#define ADDR 0x08
volatile byte sendBuffer[I2C_DATA_LEN];
byte lastBuffer[I2C_DATA_LEN];
volatile bool data_ready = false;

#define S0_PIN 0
#define S1_PIN 1
#define S2_PIN 2
#define INPUT_PIN 26
byte lineValue = 0;
Line lineMux(S0_PIN, S1_PIN, S2_PIN, INPUT_PIN);

#define LIDAR_GATE_SDA_PIN 8
#define LIDAR_GATE_SCL_PIN 9
#define LIDAR_GATE_ID 0
LidarGate lidargate(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN, LIDAR_GATE_ID);
uint8_t ballCap = 0;

uint8_t getLidarGateData(){
    ballCap = lidargate.checkBallCap();
    return ballCap;
}

uint8_t getLineData(){
    lineValue = lineMux.readData();
    return lineValue;
}

void send(){
    if(data_ready) Wire1.write((const uint8_t*)sendBuffer, I2C_DATA_LEN);
    else Wire1.write(lastBuffer, I2C_DATA_LEN);
}

void setup(){
    Serial.begin(115200);

    Wire1.setSDA(SDA_PIN);
    Wire1.setSCL(SCL_PIN);
    Wire1.begin(ADDR);
    Wire1.onRequest(send);

    lastBuffer[0] = 5;
    for (int i=1; i<I2C_DATA_LEN; i++) lastBuffer[i] = 0;

    pinMode(KICKER_LOGIC_PIN, INPUT);

    Analog_IIC_Init(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN);

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
}

void loop(){
    data_ready = false;
    sendBuffer[0] = 5;
    sendBuffer[1] = getLidarGateData();
    sendBuffer[2] = getLineData();
    data_ready = true;

    colour = (15<<8) + 15;
    if(ballCap > 0) colour += (15<<8);
    if(lineValue > 0) colour += (15<<16);
    strip.setPixelColor(0, colour);
    strip.show();

    #ifdef DEBUGGING
    for (auto i : sendBuffer){
        Serial.print(String(i) + " ");
    }
    Serial.println();
    #endif

    memcpy(&lastBuffer, (const uint8_t*) sendBuffer, I2C_DATA_LEN);
}
