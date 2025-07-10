#include <Arduino.h>
#include <Wire.h>
#include <UARTComms.h>
#include <VL53L5CX.h>
#include <Adafruit_NeoPixel.h>
#include <CommonUtils.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

#define PICO_LED 16
#define PICO_LED_BRIGHTNESS 50
Adafruit_NeoPixel pico_led(1, PICO_LED, NEO_GRB + NEO_KHZ800);

// VL53L5CX Lidars
#define VL_SDA_PIN 6
#define VL_SCL_PIN 7
#define NUM_LIDARS 6
#define SENSOR_WIDTH 8
#define SENSOR_FREQ 15
uint8_t lpin[NUM_LIDARS] = {2, 3, 10, 11, 14, 15};
uint8_t addr[NUM_LIDARS] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
std::vector<VL53L5CX> vlLidar;

// I2C Comms with ESP
#define ESP_SDA_PIN 4
#define ESP_SCL_PIN 5
#define ESP_SEND_DATA_LEN 8
#define ESP_RCV_DATA_LEN 8
#define I2C_ADDR 0x09
volatile byte espSendBuffer[ESP_SEND_DATA_LEN];
byte lastBuffer[ESP_SEND_DATA_LEN];
byte espRcvBuffer[ESP_RCV_DATA_LEN];
volatile bool data_ready = false;
byte firstbyte = 5;

// UART Comms with top camera
#define TOP_CAM_TX_PIN 8
#define TOP_CAM_RX_PIN 9
#define TOP_CAM_DATA_LEN 5
byte topCamBuffer[TOP_CAM_DATA_LEN];
UARTComms topCamUART(TOP_CAM_TX_PIN, TOP_CAM_RX_PIN, topCamBuffer, TOP_CAM_DATA_LEN, Serial2);
float top_ball_angle, top_ball_dist;
bool topNoBall = false;
float topLastSeenBall = 0;

// UART Comms with front camera
#define FRONT_CAM_TX_PIN 12
#define FRONT_CAM_RX_PIN 13
#define FRONT_CAM_DATA_LEN 9
byte frontCamBuffer[FRONT_CAM_DATA_LEN];
UARTComms frontCamUART(FRONT_CAM_TX_PIN, FRONT_CAM_RX_PIN, frontCamBuffer, FRONT_CAM_DATA_LEN, Serial1);
float front_ball_x, front_ball_y, front_ball_angle, front_ball_dist;
bool frontNoBall = true;
byte front_open = 0;
int open_rows_start = 0, open_rows_end = 0;
float frontLastSeenBall = 0;

// NeoPixel LED Strip
#define STRIP_LED_PIN 26
#define STRIP_LED_COUNT 6
#define STRIP_LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(STRIP_LED_COUNT, STRIP_LED_PIN, NEO_GRB + NEO_KHZ800);

// Variables
float self_x = 0, self_y = 0, self_heading = 0;
int rounded_ball_angle = 0, rounded_ball_dist = 0;

void send(){
    if(data_ready) Wire.write((const uint8_t*)espSendBuffer, ESP_SEND_DATA_LEN);
    else Wire.write(lastBuffer, ESP_SEND_DATA_LEN);
}

void receive(int num_bytes){
    if(num_bytes != ESP_RCV_DATA_LEN){
        // Serial.print("Received bad data length");
        return;
    }
    // else Serial.print("Received: ");
    for (int i=0; i<ESP_RCV_DATA_LEN; i++){
        if(Wire.available()){
            espRcvBuffer[i] = Wire.read();
        }
    }
    // for (auto i : espRcvBuffer){
    //     Serial.print(i);
    //     Serial.print(" ");
    // }
    // Serial.println();
    if(espRcvBuffer[0]!=firstbyte) {
        Serial.print("Received bad data");
        return;
    }
    self_x = (float)(espRcvBuffer[1] + (espRcvBuffer[2]<<8)) / 128;
    self_y = (float)(espRcvBuffer[3] + (espRcvBuffer[4]<<8)) / 128;
    self_heading = (float)(espRcvBuffer[6] + (espRcvBuffer[7]<<8)) / 128;
    if(espRcvBuffer[5]==0) self_heading *= -1;
    // DEBUG(self_x);
    // DEBUG(self_y);
    // DEBUG(self_heading);
}

void setLED(int first, int last, uint32_t color){
    for (int i=first; i<=last; i++){
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void getTopCamData(){
    bool status = topCamUART.uartRead((byte)5);
    if(status){
        top_ball_angle = (float)(topCamBuffer[1] + (topCamBuffer[2]<<8)) / 128;
        top_ball_dist = (float)(topCamBuffer[3] + (topCamBuffer[4]<<8)) / 128;

        if(top_ball_angle==0 && top_ball_dist==0) {
            topNoBall = true;
        }
        else {
            topNoBall = false;
            topLastSeenBall = millis();
        }
        // DEBUG(top_ball_angle);
        // DEBUG(top_ball_dist);
    }
}

void getFrontCamData(){
    bool status = frontCamUART.uartRead((byte)5);
    if(status){
        front_ball_x = (float)(frontCamBuffer[2] + (frontCamBuffer[3]<<8)) / 128;
        if(frontCamBuffer[1]==0) front_ball_x *= -1;
        front_ball_y = (float)(frontCamBuffer[4] + (frontCamBuffer[5]<<8)) / 128;

        if(front_ball_x==0 && front_ball_y==0) {
            frontNoBall = true;
            front_ball_angle = 0;
            front_ball_dist = 0;
        }
        else {
            frontNoBall = false;
            frontLastSeenBall = millis();
            front_ball_dist = sqrt((front_ball_x * front_ball_x) + (front_ball_y * front_ball_y));
            front_ball_angle = DEG(PI/2 - atan2(front_ball_y, front_ball_x));
            front_ball_angle = LIM_ANGLE_360(front_ball_angle);
        }
        front_open = frontCamBuffer[6];
        open_rows_start = frontCamBuffer[7];
        open_rows_end = frontCamBuffer[8];
        // DEBUG(front_ball_x);
        // DEBUG(front_ball_y);
        // DEBUG(front_ball_angle);
        // DEBUG(front_ball_dist);
    }
}

void printLidarReadings(int16_t arr[]){
    // print readings inverted (reflects reality)
    Serial.println("==================================================================");
    for (int y = 0; y <= SENSOR_WIDTH * (SENSOR_WIDTH - 1); y += SENSOR_WIDTH){
        Serial.print("||");
        for (int x = SENSOR_WIDTH - 1; x >= 0; x--){
            if(arr[x+y] < 10) Serial.print("   ");
            else if(arr[x+y] < 1000) Serial.print("  ");
            else Serial.print(" ");
            Serial.print(arr[x+y]);
            if(arr[x+y] < 100) Serial.print("  ");
            else Serial.print(" ");
            Serial.print("||");
        }
        Serial.println();
        Serial.println("==================================================================");
    }
    Serial.println();
}

void readAllLidars(){
    for (int i=0; i<NUM_LIDARS; i++){
        bool status = vlLidar[i].updateData();
        if(status){
            // setLED(i, i, strip.Color(0, 15, 0));
            Serial.printf("%d:\n", i+1);
            printLidarReadings(vlLidar[i].data.distance_mm);
        }
        // else setLED(i, i, strip.Color(15, 0, 0));
    }
}

void setup(){
    Serial.begin(115200);
    topCamUART.init();
    frontCamUART.init();

    pico_led.begin();
    pico_led.setBrightness(PICO_LED_BRIGHTNESS);
    pico_led.setPixelColor(0, pico_led.Color(15, 15, 0));
    pico_led.show();

    strip.begin();
    strip.setBrightness(STRIP_LED_BRIGHTNESS);
    // setLED(0, STRIP_LED_COUNT-1, strip.Color(0, 0, 15));

    for (uint8_t i=0; i<NUM_LIDARS; i++){
        vlLidar.emplace_back(VL_SCL_PIN, VL_SDA_PIN, lpin[i], addr[i], SENSOR_WIDTH, SENSOR_FREQ, Wire1);
        pinMode(lpin[i], OUTPUT);
        digitalWrite(lpin[i], LOW);
    }

    vlLidar[0].initWire();

    for (uint8_t i=0; i<NUM_LIDARS; i++) vlLidar[i].init();

    Wire.setSDA(ESP_SDA_PIN);
    Wire.setSCL(ESP_SCL_PIN);
    Wire.begin(I2C_ADDR);
    Wire.onRequest(send);
    Wire.onReceive(receive);

    lastBuffer[0] = 5;
    for (int i=1; i<ESP_SEND_DATA_LEN; i++) lastBuffer[i] = 0;
    espSendBuffer[0] = 5;
}

void loop(){
    pico_led.setPixelColor(0, pico_led.Color(15, 0, 15));
    pico_led.show();
    getTopCamData();
    getFrontCamData();

    data_ready = false;
    if(!frontNoBall){
        rounded_ball_angle = floor(front_ball_angle * 128);
        rounded_ball_dist = floor(front_ball_dist * 128);
    }
    else{
        rounded_ball_angle = floor(top_ball_angle * 128);
        rounded_ball_dist = floor(top_ball_dist * 128);
    }
    espSendBuffer[0] = 5;
    espSendBuffer[1] = rounded_ball_angle & 0xFF;
    espSendBuffer[2] = (rounded_ball_angle >> 8) & 0xFF;
    espSendBuffer[3] = rounded_ball_dist & 0xFF;
    espSendBuffer[4] = (rounded_ball_dist >> 8) & 0xFF;
    espSendBuffer[5] = front_open;
    espSendBuffer[6] = open_rows_start;
    espSendBuffer[7] = open_rows_end;
    data_ready = true;

    memcpy(&lastBuffer, (const uint8_t*) espSendBuffer, ESP_SEND_DATA_LEN);

    readAllLidars();
}
