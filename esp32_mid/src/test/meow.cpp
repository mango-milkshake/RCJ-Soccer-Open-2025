#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Kicker.h>
#include <cmath>
#include <algorithm>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

//// ** DEFINITIONS ** ////

// ESP NeoPixel LED
#define ESP_LED 21
#define ESP_BRIGHTNESS 50
#define BLINK_TIME 10
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);
// to check if code is running
bool esp_led_state = true;
float lastLED = 0;

// Debug LEDs
#define LED_PIN 18
#define LED_COUNT 12
#define LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor software switch
#define TURN_OFF_SW 40
bool turnOff = false;

// Voltage checker
#define VOLTAGE_PIN 2
#define VOLTAGE_ANALOG_THRESH 3300

// Dimensions
#define FIELD_WIDTH 1.82
#define FIELD_HEIGHT 2.43
#define BALL_CAP_THRESH 15 // in cm
#define BOT_RADIUS 8.5 // in cm

// I2C Comms with bottom plate
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_RCV_DATA_LEN 16
#define I2C_SEND_DATA_LEN 7
#define I2C_RCV_PICO_ADDR 0x08
#define I2C_SEND_PICO_ADDR 0x09

#define FRONT_CAM_DATA_POS 1
#define BALLCAP_LIDAR_POS 6
#define LIDAR_GATE_POS 11
#define LINE_DATA_POS 12

byte rcvBuffer[I2C_RCV_DATA_LEN+1], sendBuffer[I2C_SEND_DATA_LEN];
byte zeroBuffer[I2C_SEND_DATA_LEN];

// UART Comms with top plate
#define PICO_TX_PIN 16
#define PICO_RX_PIN 17
#define PICO_SERIAL_DATA_LEN 9
byte uartBufferPico[PICO_SERIAL_DATA_LEN];

// UART Comms with camera
#define CAM_TX_PIN 10
#define CAM_RX_PIN 11
#define CAM_SERIAL_DATA_LEN 9
byte uartBufferCam[CAM_SERIAL_DATA_LEN];

// PID
PID pid_rotate(0.2, 0, 0, 1000);
PID pid_x(2, 0, 0, 5000);
PID pid_y(2, 0, 0, 5000);

// Dribbler
#define MOSI_PIN 12
#define MISO_PIN 13
#define SCK_PIN 14
#define CS_PIN 15
#define DRIBBLER_IN1 34
#define DRIBBLER_IN2 33
#define DRIBBLER_NFAULT 35
#define NSLEEP_PIN 36
#define DRVOFF_PIN 37
MotorDriver dribblerMD(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

uint8_t dribbler_maxspeed = 80;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
float lastFault = 0;

// Kicker
#define KICKER_PIN 42
Kicker kicker(KICKER_PIN);

// Line Sensors
#define NUM_LINE_MUX 4

// Thresholds
#define BALLCAP_DURATION 250
#define ALIGNED_THRESHOLD 0.015f
#define MOVING_BACK_DURATION 200
#define INITIAL_CHANGE 35.0f
#define GRADUAL_CHANGE 250.0f
#define ALIGN_DURATION 2000
#define ALIGN_THRESHOLD 3000
#define BALLCAP_DISTANCE 0.02f
#define BALLCAP_WIDTH 0.0335f
#define CLEARANCE_X 0.20f
#define CLEARANCE_Y 0.15f
#define FIELD_MARGIN 0.12f
#define FIELD_MARGIN_X 0.51f
#define FIELD_MARGIN_Y 0.37f

// Variables
float self_x = 0, self_y = 0, self_heading = 0;
float ball_angle = 0, ball_dist = 0;
float ball_vx = 0, ball_vy = 0;
float ball_x_topcam = 0, ball_y_topcam = 0;
float ball_vx_topcam = 0, ball_vy_topcam = 0;
float relative_ball_x = 0, relative_ball_y = 0;
float absolute_ball_x = 0, absolute_ball_y = 0;
float abs_bx_front = 0, abs_by_front = 0;
float ball_vx_front = 0, ball_vy_front = 0;
float ball_x_frontcam = 0, ball_y_frontcam = 0;
float line_status[NUM_LINE_MUX];
bool noBall = false, ballCap = false, isTilted = false, isOnLine = false;
float lastLoopTime = 0, lastBallCap = 0;
float speed_xdir, speed_ydir, rotation;
float other_x = 0, other_y = 0;
bool isDefender = true;

uint8_t broadcastAddress[6] = {0,0,0,0,0,0}; 
typedef struct struct_message {
    bool def;
    float xpos; 
    float ypos;
    bool hasBall; 
} struct_message;
struct_message espnowData;

bool moving_back = false;
unsigned long last_moving_back = 0;
bool aligned = false;
float initial_change = 0.0f, initial_magnitude = 0.0f;
unsigned long last_aligning = 0;

//// ** FUNCTIONS ** ////

void setLED(int first, int last, uint32_t color){
    for (int i=first; i<=last; i++){
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void readVoltage(){
    float value = analogRead(VOLTAGE_PIN);
    if(value < VOLTAGE_ANALOG_THRESH) setLED(0, LED_COUNT, strip.Color(50, 0, 0));
}

void checkFault(){
    bool faulted = false;
    if(digitalRead(dribbler.nfault)==LOW) faulted = true;
    if(faulted){
        // Serial.println("faulted");
        setLED(0, 0, strip.Color(15, 15, 15));
        float curTime = millis();
        if(curTime - lastFault >= 500){
            dribblerMD.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
    else setLED(0, 0, strip.Color(0, 0, 0));
}

void readMacAddress(){ //read own mac address and set broadcast address to other bot
    uint8_t own_mac_address[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, own_mac_address);
    if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  own_mac_address[0], own_mac_address[1], own_mac_address[2],
                  own_mac_address[3], own_mac_address[4], own_mac_address[5]);
    } 
    else{
        Serial.println("Failed to read MAC address");
    }
    if(memcmp(own_mac_address, (uint8_t[]){0x34, 0x85, 0x18, 0xbc, 0xe0, 0x60}, 6) == 0) {
        memcpy(broadcastAddress, (uint8_t[]){0x34, 0x85, 0x18, 0xbc, 0xe0, 0x40}, 6);
   }
    else{
        memcpy(broadcastAddress, (uint8_t[]){0x34, 0x85, 0x18, 0xbc, 0xe0, 0x60}, 6);
    } 
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){  
    //Serial.print("\r\nLast Packet Send Status:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){ //interpret received data here
    memcpy(&espnowData, incomingData, sizeof(espnowData));

}

void set_up_esp_now(){
    esp_now_peer_info_t peerInfo = {};
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    //callback functions for sending and receiving
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
}

void sendData(){ //send data here
    //Define what values to send
    espnowData.def = isDefender ? true : false;
    espnowData.xpos = self_x;
    espnowData.ypos = self_y;
    espnowData.hasBall = ballCap ? true : false;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &espnowData, sizeof(espnowData));        
    if (result == ESP_OK) {
        //Serial.println("Sent with success");
    }
    else {
        //Serial.println("Error sending the data");
    }
}


void getTopPlateData(){
    if(Serial2.available()>=PICO_SERIAL_DATA_LEN){
        while(Serial2.available()>=PICO_SERIAL_DATA_LEN && Serial2.peek()!=5) {
            Serial.println("Pico first byte not 5");
            Serial2.read();
        }
        int len = Serial2.readBytes(uartBufferPico, PICO_SERIAL_DATA_LEN);
        while(Serial2.available()) Serial2.read();
        if(len!=PICO_SERIAL_DATA_LEN || uartBufferPico[0]!=5){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : uartBufferPico) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            self_x = (float)(uartBufferPico[1] + (uartBufferPico[2]<<8)) / 128;
            self_y = (float)(uartBufferPico[3] + (uartBufferPico[4]<<8)) / 128;
            self_heading = (float)(uartBufferPico[6] + (uartBufferPico[7]<<8)) / 128;
            if(uartBufferPico[5]==0) self_heading *= -1;
            if(uartBufferPico[8]==1) isTilted = true;
            else isTilted = false;
            setLED(1, 2, strip.Color(15, 0, 15));
            DEBUG(self_x);
            DEBUG(self_y);
            DEBUG(self_heading);
        }
    }
    else setLED(1, 2, strip.Color(0, 15, 15));
}

void getTopCamData(){
    if(Serial1.available()>=CAM_SERIAL_DATA_LEN){
        while(Serial1.available()>=CAM_SERIAL_DATA_LEN && Serial1.peek()!=1) {
            Serial.println("Camera first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(uartBufferCam, CAM_SERIAL_DATA_LEN);
        while(Serial1.available()) Serial1.read();
        if(len!=CAM_SERIAL_DATA_LEN || uartBufferCam[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : uartBufferCam) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{ 
            ball_x_topcam = (float)(uartBufferCam[1] + (uartBufferCam[2]<<8)) / 128;
            ball_y_topcam = (float)(uartBufferCam[3] + (uartBufferCam[4]<<8)) / 128;
            ball_vx_topcam = (float)(uartBufferCam[5] + (uartBufferCam[6]<<8)) / 128;
            ball_vy_topcam = (float)(uartBufferCam[7] + (uartBufferCam[8]<<8)) / 128;
            if(ball_x_topcam==0 && ball_y_topcam==0) {
                noBall = true;
                setLED(6, 8, strip.Color(0, 0, 15));
            }
            else {  
                noBall = false;
                setLED(6, 8, strip.Color(0, 15, 0)); //green
            }

            if(noBall) dribbler.setSpeed(0);
            else if(ball_dist<=40) dribbler.setSpeed(1.0);
            else dribbler.setSpeed(0.5);

            relative_ball_x = ball_y_topcam; //rotate -90
            relative_ball_y = ball_x_topcam;
            ball_vx = ball_vy_topcam;
            ball_vy = ball_vx_topcam;


            //float relative_angle = 90 - (ball_angle + self_heading);
            // ball_dist += BOT_RADIUS;
            //relative_ball_x = (ball_dist * cosf(RAD(relative_angle))) / 100;
            //relative_ball_y = (ball_dist * sinf(RAD(relative_angle))) / 100;
            absolute_ball_x = relative_ball_x + self_x;
            absolute_ball_y = relative_ball_y + self_y;

            DEBUG(relative_ball_x);
            DEBUG(relative_ball_y);
            DEBUG(absolute_ball_x);
            DEBUG(absolute_ball_y);
        }
    }
}

void getBottomPlateData(){
    byte num_bytes = Wire.requestFrom(I2C_RCV_PICO_ADDR, I2C_RCV_DATA_LEN);
    if(num_bytes != I2C_RCV_DATA_LEN){
        Serial.print("Received bad data: ");
        setLED(3, 4, strip.Color(15, 15, 0));
    }
    else Serial.print("Received: ");
    for (int i=0; i<I2C_RCV_DATA_LEN; i++) {
        if (Wire.available()) {
            rcvBuffer[i] = Wire.read();
        }
    }
    rcvBuffer[I2C_RCV_DATA_LEN] = '\0';
        
    if(rcvBuffer[0]!=1) {
        Serial.println("Bad data received");
        setLED(3, 4, strip.Color(15, 15, 0));
    }
    else setLED(3, 4, strip.Color(0, 15, 0));

    ball_x_frontcam = (float)(rcvBuffer[2] + (rcvBuffer[3]<<8)) / 128;
    if(rcvBuffer[1]==0) ball_x_frontcam *= -1;
    ball_y_frontcam = (float)(rcvBuffer[4] + (rcvBuffer[5]<<8)) / 128;
    ball_vx_front = (float)(rcvBuffer[6] + (rcvBuffer[7]<<8)) / 128;
    ball_vy_front = (float)(rcvBuffer[8] + (rcvBuffer[9]<<8)) / 128;

    abs_bx_front = ball_x_frontcam + self_x;
    abs_by_front = ball_y_frontcam + self_y;

    // lidar_ball_x = (float)(rcvBuffer[7] + (rcvBuffer[8]<<8)) / 128;
    // if(rcvBuffer[6]==0) lidar_ball_x *= -1;
    // lidar_ball_y = (float)(rcvBuffer[9] + (rcvBuffer[10]<<8)) / 128;

    // DEBUG(cam_ball_x);
    // DEBUG(cam_ball_y);
    // DEBUG(lidar_ball_x);
    // DEBUG(lidar_ball_y);

    ballCap = (bool) rcvBuffer[LIDAR_GATE_POS];

    isOnLine = false;
    for (uint8_t i=0; i<4; i++) {
        line_status[i] = rcvBuffer[LINE_DATA_POS+i];
        if(line_status[i]>0) isOnLine = true;
    }
    if(isOnLine) setLED(5, 5, strip.Color(15, 15, 15));
    else setLED(5, 5, strip.Color(0, 0, 0));
}

void ballCapStatus(){
    if(ballCap){
        setLED(9, 11, strip.Color(15, 0, 15));
        lastBallCap = millis();
    }
    else if(!noBall && self_y < absolute_ball_y && self_y > absolute_ball_y - BALLCAP_DISTANCE 
        && abs(relative_ball_x) < BALLCAP_WIDTH / 2.0){
            ballCap = true;
            lastBallCap = millis();
            setLED(9, 11, strip.Color(0, 15, 15));
    }
    else if(millis() - lastBallCap < BALLCAP_DURATION){
        ballCap = true;
        setLED(9, 11, strip.Color(15, 15, 15));
    }
    else setLED(9, 11, strip.Color(15, 15, 0));
}

void sendI2C(byte (&buffer)[I2C_SEND_DATA_LEN]){
    Wire.beginTransmission(I2C_SEND_PICO_ADDR);
    Wire.write(buffer, I2C_SEND_DATA_LEN);
    Wire.endTransmission();
}

void movement(float target_x, float target_y, float target_rotation){
    target_x = constrain(target_x, 0.12, FIELD_WIDTH - 0.12);
    target_y = constrain(target_y, 0.37, FIELD_HEIGHT - 0.37);
    float x_dist = target_x - self_x, y_dist = target_y - self_y;
    float total_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    float total_angle = atan2(y_dist, x_dist) + RAD(self_heading) - PI/4; // in radians

    float rotation_dist = self_heading - target_rotation;
    while(rotation_dist > 180) rotation_dist -= 360;
    while(rotation_dist < -180) rotation_dist += 360;

    float shifted_x_dist = total_dist * sinf(total_angle);
    float shifted_y_dist = total_dist * cosf(total_angle);

    speed_xdir = pid_x.compute(0, shifted_x_dist);
    speed_ydir = pid_y.compute(0, shifted_y_dist);
    rotation = constrain(pid_rotate.compute(0, RAD(rotation_dist)), -1, 1);

    float maxPID = max(abs(speed_xdir), abs(speed_ydir));
    if(maxPID > 1){
        float k = 1/maxPID;
        speed_xdir *= k;
        speed_ydir *= k;
    }

    uint8_t rotation_sign, speed_x_sign, speed_y_sign;
    if(copysign(1, rotation)==1) rotation_sign = 1;
    else rotation_sign = 0;
    if(copysign(1, speed_xdir)==1) speed_x_sign = 1;
    else speed_x_sign = 0;
    if(copysign(1, speed_ydir)==1) speed_y_sign = 1;
    else speed_y_sign = 0;
    uint8_t rounded_rotation = floor(abs(rotation) * 255);
    uint8_t rounded_speed_x = floor(abs(speed_xdir) * 255);
    uint8_t rounded_speed_y = floor(abs(speed_ydir) * 255);

    sendBuffer[0] = 5;
    sendBuffer[1] = speed_x_sign;
    sendBuffer[2] = rounded_speed_x;
    sendBuffer[3] = speed_y_sign;
    sendBuffer[4] = rounded_speed_y;
    sendBuffer[5] = rotation_sign;
    sendBuffer[6] = rounded_rotation;

    if(turnOff || isTilted) sendI2C(zeroBuffer);
    else sendI2C(sendBuffer);
}

// void ballTrack(){
//     float absBallAngle = atan2(relative_ball_y - self_y, relative_ball_x - self_x);
//     float goalToBallAngle = atan2(2.384 - relative_ball_y, 0.91 - relative_ball_x);
//     float angleToFace = atan2(2.384 - self_y, 0.91 - self_x);
//     if(absBallAngle <= 60 || absBallAngle >= 300){
//         // movement(self_x + relative_ball_x, self_y + relative_ball_y - 0.06, 90-DEG(angleToFace));
//         movement(self_x + relative_ball_x, self_y + relative_ball_y - 0.04, 0);
//     }
//     else{
//         float new_x = self_x + relative_ball_x + 0.40 * cosf(goalToBallAngle);
//         float new_y = self_y + relative_ball_y + 0.40 * sinf(goalToBallAngle);
//         movement(new_x, new_y, 90-DEG(angleToFace));
//         // movement(self_x + relative_ball_x, self_y + relative_ball_y - 0.40, 90-DEG(angleToFace));
//         // movement(self_x + relative_ball_x, self_y + relative_ball_y - 0.40, 0);
//     }
//     // movement(self_x + relative_ball_x, self_y + relative_ball_y - 0.12, 90-DEG(angleToFace));
// }

void ballTrack(){
    aligned = false;
    initial_change = 0.0;
    initial_magnitude = 0.0;

    float new_x, new_y;
    if(self_y > absolute_ball_y) moving_back = true;
    if((moving_back || millis() - last_moving_back > MOVING_BACK_DURATION) && 
        (self_y > absolute_ball_y - BALLCAP_DISTANCE / 3.0 || 
        (abs(self_x - absolute_ball_x) > BALLCAP_WIDTH / 2.0 + 0.05f && 
        abs(self_x - absolute_ball_x) < CLEARANCE_X / 2.0 && 
        self_y > absolute_ball_y - CLEARANCE_Y / 2.0))){
            // Serial.println("case1");
            if(absolute_ball_x < FIELD_MARGIN + CLEARANCE_X + 0.10f) new_x = absolute_ball_x + (CLEARANCE_X / 2.0 + 0.05f);
            else if(absolute_ball_x > FIELD_WIDTH - FIELD_MARGIN - CLEARANCE_X - 0.10f) new_x = absolute_ball_x - (CLEARANCE_X / 2.0 + 0.05f);
            else if (self_x > absolute_ball_x) new_x = absolute_ball_x + (CLEARANCE_X / 2.0 + 0.05f);
            else new_x = absolute_ball_x - (CLEARANCE_X / 2.0 + 0.05f);
            new_y = (abs(self_x - absolute_ball_x) > CLEARANCE_X / 2.0 + 0.03f) ? absolute_ball_y - CLEARANCE_Y / 2.0 - 0.10f : self_y;
            moving_back = true;
    }
    else{
        // Serial.println("case2");
        if(moving_back) {
            moving_back = false;
            last_moving_back = millis();
        }
        unsigned long aligning = millis() - last_aligning;
        new_x = absolute_ball_x;
        if((aligning > ALIGN_DURATION && aligning < ALIGN_THRESHOLD) || abs(self_x - absolute_ball_x) < BALLCAP_WIDTH / 2.0) 
            new_y = fmax(absolute_ball_y - BALLCAP_DISTANCE, self_y + 0.03f) ;
        else{
            if (aligning > ALIGN_THRESHOLD) last_aligning = millis(); 
            new_y = absolute_ball_y - BALLCAP_DISTANCE;
        }
    }
    DEBUG(new_x);
    DEBUG(new_y);
    movement(new_x, new_y, 0);
}

void aim(){
    if(!aligned){
        // Serial.println("aim align");
        if(abs(self_x - absolute_ball_x) < ALIGNED_THRESHOLD) aligned = true;
        DEBUG(self_x);
        DEBUG(absolute_ball_x);
        movement(absolute_ball_x, self_y, 0);
    }
    else{
        // Serial.println("aim accel");
        float xToGoal = 0.91 - self_x, yToGoal = 2.384 - self_y;
        float distToGoal = sqrt(xToGoal * xToGoal + yToGoal * yToGoal);
        float angleToGoal = PI/2 - atan2(yToGoal, xToGoal);
        if(initial_change == 0){
            initial_magnitude = distToGoal;
            initial_change = max(0.0f, cosf(angleToGoal)) * INITIAL_CHANGE;
        }
        float change = initial_change + max(0.0f, initial_magnitude - distToGoal) / initial_magnitude * GRADUAL_CHANGE;
        change = min(change, max(0.0f, (self_y + FIELD_MARGIN_Y)*100/cosf(angleToGoal)));
        float new_x = self_x + change * sinf(angleToGoal) / 100;
        float new_y = self_y + change * cosf(angleToGoal) / 100;
        DEBUG(change);
        DEBUG(new_x);
        DEBUG(new_y);
        movement(new_x, new_y, DEG(angleToGoal));
    }
    // float angleToFace = atan2(2.384 - self_y, 0.91 - self_x);
    // movement(0.91, 2.06, 90-DEG(angleToFace));
    if(ballCap && self_y > 1.83 && (self_heading > -75 && self_heading < 75)) kicker.kick();
}

void frontCamTrack(){ //turns bot to ball based on top cam, use if a more accurate front cam measurement is needed
    float approx_future_x = absolute_ball_x + ball_vx * 0.2;
    float approx_future_y = absolute_ball_y + ball_vy * 0.2;
    movement(self_x, self_y, atan2(approx_future_x, approx_future_y));
}

void lookAhead(){
    float v = 0.5;
    float latency = 0.1;
    bool validt = false;
    bool useFrontCam = false;
    float t;
    float LAball_x, LAball_y, LAball_vx, LAball_vy;
    
    if (useFrontCam){
        frontCamTrack();
        LAball_x = abs_bx_front + ball_vx * latency;
        LAball_y = abs_by_front + ball_vy * latency;
        LAball_vx = ball_vx_front;
        LAball_vy = ball_vy_front;
    }
    else{
        LAball_x = absolute_ball_x + ball_vx * latency;
        LAball_y = absolute_ball_y + ball_vy * latency;
        LAball_vx = ball_vx;
        LAball_vy = ball_vy;
    }

    while(!validt){

        float C = LAball_x*LAball_x + LAball_y*LAball_y;
        float B =  2*(LAball_x*ball_vx + LAball_y*LAball_vy);
        float A = LAball_vx*LAball_vx + LAball_vy*LAball_vy - v*v;

        if (abs(A) > pow(10, -8 )){ //we get two solutions for time, so we want to find the minimum time that is not negative
            float t1 = pow(-1*B - (B*B - 4*A*C), 0.5)/(2*A); 
            float t2 = pow(-1*B + (B*B - 4*A*C), 0.5)/(2*A); 

            if (t1 >= 0 && t2 >= 0){
                t = min(t1, t2);
                validt = true;
            }
            else if (t1 >= 0){
                t = t1;
                validt = true;
            }
            else if (t2 >= 0){
                t = t2;
                validt = true;
            }  
        }

        if(!validt){ //if ball is too fast, reduce ball's velocity and calculate that position instead
            float theta;
            if ((LAball_x + LAball_vx*t) > pow(10, -8)){ 
                theta = atan(LAball_vx/LAball_vy);}
            else{theta = 3.1415/2;}
            LAball_vx = 0.9*v*cos(theta); // if magnitude of ball's velocity is less than bot's velocity, it should be interceptable regardless of direction
            LAball_vx = 0.9*v*sin(theta);
        }  
    }
    //assume front is facing towards positive y
    float targetballposx = LAball_x + LAball_vx*t;
    float targetballposy = LAball_y + LAball_vy*t;
    float targetheadinglookahead = atan2(targetballposx,targetballposy) * (180/3.1415) + 90; 
    targetballposx = targetballposx + self_x;
    targetballposy = targetballposx + self_y;

    movement(targetballposx, targetballposy, targetheadinglookahead);
}


//// ** LOOPS ** ////

void setup(){
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    Serial2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    analogSetAttenuation(ADC_11db);

    Wire.begin(SDA_PIN, SCL_PIN, 50000);

    for (int i=0; i<I2C_SEND_DATA_LEN; i++) zeroBuffer[i] = 0;

    dribblerMD.init();
    dribblerMD.setMode();

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

    set_up_esp_now();
    readMacAddress();
}

void loop(){
    Serial.println("running main code");
    float curTime = millis();
    Serial.print("time: ");
    Serial.println(curTime - lastLoopTime);
    lastLoopTime = millis();

    if(millis() - lastLED >= BLINK_TIME){
        esp_led_state = !esp_led_state;
        lastLED = millis();
    }
    if(esp_led_state) esp_led.setPixelColor(0, esp_led.Color(0, 50, 0));
    else esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
    esp_led.show();

    if(digitalRead(TURN_OFF_SW)==HIGH) turnOff = true;
    else turnOff = false;

    // readVoltage();
    checkFault();
    getTopPlateData();
    getTopCamData();
    getBottomPlateData();
    ballCapStatus();

    sendData();

    if(noBall) movement(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
    else if(ballCap) aim();
    else if (ball_vx > 10 || ball_vy > 10) lookAhead();
    else ballTrack();
}
