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

#define SDA_PIN 6
#define SCL_PIN 7
#define I2C_DATA_LEN 16
#define ADDR 0x08
byte sendBuffer[I2C_DATA_LEN];
#define FRONT_CAM_DATA_POS 1
#define BALLCAP_LIDAR_POS 6
#define LIDAR_GATE_POS 11
#define LINE_DATA_POS 12
// 0: start byte = 1
// 1-5: front cam
// 6-10: ballcap lidar
// 11: tofsense lidar gate
// 12-15: line sensors

#define NUM_MUX 4
#define S0_PIN 0
#define S1_PIN 1
#define S2_PIN 2
#define INPUT_PIN_POS 26
std::vector<Line> lineMux;

#define CAM_TX_PIN 12
#define CAM_RX_PIN 13
#define CAM_DATA_LEN 6
byte camBuffer[CAM_DATA_LEN];
float cam_ball_x, cam_ball_y;

#define LIDAR_TX_PIN 4
#define LIDAR_RX_PIN 5
#define LIDAR_DATA_LEN 6
byte lidarBuffer[LIDAR_DATA_LEN];
float lidar_ball_x, lidar_ball_y;

#define LIDAR_GATE_SDA_PIN 8
#define LIDAR_GATE_SCL_PIN 9
#define LIDAR_GATE_ID 0
LidarGate lidargate(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN, LIDAR_GATE_ID);

void getAllLineData(){
    for (int i=0; i<4; i++){
        sendBuffer[LINE_DATA_POS+i] = lineMux[i].readData();
    }
}

void getCamData(){
    if(Serial1.available()>=CAM_DATA_LEN){
        while(Serial1.available()>=CAM_DATA_LEN && Serial1.peek()!=1) {
            Serial.println("Camera first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(camBuffer, CAM_DATA_LEN);
        Serial.println(len);
        if(len!=CAM_DATA_LEN || camBuffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : camBuffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            for (int i=0; i<5; i++) sendBuffer[FRONT_CAM_DATA_POS+i] = camBuffer[i+1];
        }
    }
}

void getBallCapData(){
    if(Serial2.available()>=LIDAR_DATA_LEN){
        while(Serial2.available()>=LIDAR_DATA_LEN && Serial2.peek()!=1) {
            Serial.println("Lidar first byte not 1");
            Serial2.read();
        }
        int len = Serial2.readBytes(lidarBuffer, LIDAR_DATA_LEN);
        Serial.println(len);
        if(len!=LIDAR_DATA_LEN || lidarBuffer[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : lidarBuffer) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            for (int i=0; i<5; i++) sendBuffer[BALLCAP_LIDAR_POS+1] = lidarBuffer[i+1];
        }
    }
}

bool getLidarGateData(){
    return lidargate.checkBallCap();
}

void send(){
    // sendBuffer[0] = 1;
    // getCamData();
    // getBallCapData();
    // sendBuffer[11] = 0; // not done
    // getAllLineData();

    #ifdef DEBUGGING
    for (auto i : sendBuffer){
        Serial.print(String(i) + " ");
    }
    Serial.println();
    #endif

    Wire1.write(sendBuffer, I2C_DATA_LEN);
}

void setup(){
    Serial.begin(115200);

    Serial1.setTX(CAM_TX_PIN);
    Serial1.setRX(CAM_RX_PIN);
    Serial1.begin(115200);

    Serial2.setTX(LIDAR_TX_PIN);
    Serial2.setRX(LIDAR_RX_PIN);
    Serial2.begin(115200);

    Wire1.setSDA(SDA_PIN);
    Wire1.setSCL(SCL_PIN);
    Wire1.setClock(400000);
    Wire1.begin(ADDR);
    Wire1.onRequest(send);

    Analog_IIC_Init(LIDAR_GATE_SCL_PIN, LIDAR_GATE_SDA_PIN);

    for (uint8_t i=0; i<NUM_MUX; i++){
        lineMux.emplace_back(S0_PIN, S1_PIN, S2_PIN, INPUT_PIN_POS+i);
    }

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
}

void loop(){
    strip.setPixelColor(0, strip.Color(0, 15, 15));
    strip.show();

    sendBuffer[0] = 1;
    getCamData();
    getBallCapData();
    sendBuffer[LIDAR_GATE_POS] = (uint8_t) getLidarGateData(); // not done
    getAllLineData(); 
}
