#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Kicker.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

// #define SECOND_BOT
//  #define LOOK_AHEAD
// #define NO_DRIBBLER

//// ** DEFINITIONS ** ////

// ESP NeoPixel LED
#define ESP_LED 21
int ESP_BRIGHTNESS = 50;
#define BLINK_TIME 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);
// to check if code is running
bool esp_led_state = true;
float lastLED = 0;

// Debug LEDs
#define LED_PIN 18
#define LED_COUNT 12
int LED_BRIGHTNESS = 100;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Switches
// Motor software switch
#define TURN_OFF_SW 40
bool turnOff = false;
// state switch
#define STATE_SW 41
#define STATE_SWAP_TIME 1000
#define TOTAL_STATES 3
float lastStateSwap = 0;
int codeState = 0; // 0 = dribbler + ball hide, 1 = dribbler + normal scoring, 2 = without dribbler
// pause switches
#define PAUSE_SW1 38
#define PAUSE_SW2 39

// Voltage checker
#define VOLTAGE_PIN 2
#define VOLTAGE_ANALOG_THRESH 3300

// Dimensions
#define FIELD_WIDTH 1.82
#define FIELD_HEIGHT 2.43
#define SELF_GOAL_LEFT_X 0.61
#define SELF_GOAL_RIGHT_X 1.21
#define SELF_GOAL_Y 0.12
#define OPP_GOAL_CENTRE_X 0.91
#define OPP_GOAL_CENTRE_Y 2.384
#define OPP_GOAL_MIDDLE_X 0.91
#define OPP_GOAL_MIDDLE_Y 2.06
#define OPP_GOAL_LEFT_X 0.61
#define OPP_GOAL_RIGHT_X 1.21
#define OPP_GOAL_Y 2.31
#define BOT_RADIUS_CM 8.5 // in cm
#define BOT_RADIUS_M 0.085 // in metres
#define Y_BOUND 1.50 

// I2C Comms with bottom plate
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_RCV_DATA_LEN 18
#define I2C_SEND_DATA_LEN 7
#define I2C_RCV_PICO_ADDR 0x08
#define I2C_SEND_PICO_ADDR 0x09

#define FRONT_CAM_DATA_POS 1
#define LIDAR_GATE_POS 13
#define LINE_DATA_POS 14

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
#define CAM_SERIAL_DATA_LEN 11
byte uartBufferCam[CAM_SERIAL_DATA_LEN];

// PID
float pid_def_rotate_default[3] = {0.5, 0, 0};
float pid_def_x_default[3] = {2.5, 0, 0};
float pid_def_y_default[3] = {2.5, 0, 0};

float pid_att_rotate_default[3] = {0.3, 0, 0};
float pid_att_x_default[3] = {2.2, 0, 0};
float pid_att_y_default[3] = {2.2, 0, 0};

PID pid_rotate(pid_att_rotate_default[0], pid_att_rotate_default[1], pid_att_rotate_default[2], 1000);
PID pid_x(pid_att_x_default[0], pid_att_x_default[1], pid_att_x_default[2], 1000);
PID pid_y(pid_att_y_default[0], pid_att_y_default[1], pid_att_y_default[2], 1000);
float max_translation_pid_value = 1, max_rotation_pid_value = 1;

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

uint8_t dribbler_maxspeed = 120;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
float lastFault = 0;

// Kicker
#define KICKER_PIN 42
Kicker kicker(KICKER_PIN);

// Line Sensors
#define NUM_LINE_MUX 4

// ESP Bluetooth Communication
uint8_t broadcastAddress[6] = {0,0,0,0,0,0}; 
uint8_t own_mac_address[6];
typedef struct struct_message {
    int isPresent;
    bool def;
    float xpos; 
    float ypos;
    bool hasBall; 
    bool inField;
} struct_message;
struct_message espnowData;
struct_message espnowDataRecv;

// Thresholds
#define BALLCAP_DURATION 250
#define ALIGNED_THRESHOLD 0.015f
#define MOVING_BACK_DURATION 200
#define INITIAL_CHANGE 35.0f
#define GRADUAL_CHANGE 250.0f
#define ALIGN_DURATION 2000
#define ALIGN_THRESHOLD 3000
#define BALLCAP_DISTANCE -0.10f
#define BALLCAP_WIDTH 0.0335f
#define CLEARANCE_X 0.20f
#define CLEARANCE_Y 0.15f
#define FIELD_MARGIN 0.12f
#define FIELD_MARGIN_X 0.51f
#define FIELD_MARGIN_Y 0.37f
#define LAST_SEEN_BALL_TIME 1000
#define SCORING_WAIT_TIME 500
#define DEFENDER_WAIT_TIME 1500
#define DEFENDER_MAX_YPOS 0.80
#define ATTACKER_MIN_BALL_YPOS 0.80
#define OSCILLATE_WAIT_TIME 2000

// Variables
float self_x = 0, self_y = 0, self_heading = 0;
float top_ball_angle = 0, top_ball_dist = 0;
float ball_vx = 0, ball_vy = 0;
float top_ball_vx = 0, top_ball_vy = 0;
float top_relative_ball_x = 0, top_relative_ball_y = 0;
float top_absolute_ball_x = 0, top_absolute_ball_y = 0;
float front_ball_x = 0, front_ball_y = 0, front_ball_vx = 0, front_ball_vy = 0;
float front_relative_ball_x = 0, front_relative_ball_y = 0;
float front_absolute_ball_x = 0, front_absolute_ball_y = 0;
float final_ball_dist = 0, final_absolute_ball_x = 0, final_absolute_ball_y = 0;
float last_ball_x = 0, last_ball_y = 0;
int current_target_x = 999, current_target_y = 999;
float self_velocityx = 0, self_velocityy = 0;
float last_self_x = 0, last_self_y = 0;
unsigned long last_vel_time = 0;
bool otherBotExists = false;
bool reachTargetOscillation = false;
float reachOscillatePointTime = 0;

float line_status[NUM_LINE_MUX];
bool noBall = false, ballCap = false, isTilted = false, isOnLine = false;
bool oscillateState = false, ballHideState = false;
float lastLoopTime = 0, lastBallCap = 0, lastNoBallCap = 0, lastSeenBall = millis();
float speed_xdir, speed_ydir, rotation;
float lastDribblerRev = 0;
float other_x = 0, other_y = 0;
bool isDefender = true;
bool lookingAhead = false;

bool moving_back = false;
unsigned long last_moving_back = 0;
bool aligned = false;
float initial_change = 0.0f, initial_magnitude = 0.0f;
unsigned long last_aligning = 0;
bool time_to_score = false;

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
            dribblerMD.clearFault();
            dribblerMD.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
    else setLED(0, 0, strip.Color(0, 0, 0));
}

void readMacAddress(){ //read own mac address and set broadcast address to other bot
    WiFi.mode(WIFI_STA);
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, own_mac_address);
    if (ret == ESP_OK) {
    // Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
    //               own_mac_address[0], own_mac_address[1], own_mac_address[2],
    //               own_mac_address[3], own_mac_address[4], own_mac_address[5]);
    // } 
    // else{
    //     Serial.println("Failed to read MAC address");
    // }
    //const uint8_t MAC_1[6] = {0x34, 0x85, 0x18, 0xbc, 0xe0, 0x60}; //cooked
    const uint8_t MAC_1[6] = {0x34, 0x85, 0x18, 0xbc, 0xe0, 0x40};
    const uint8_t MAC_2[6] = {0x34, 0x85, 0x18, 0xbc, 0xf5, 0xe8};
    //const uint8_t MAC_3[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 
    if (memcmp(own_mac_address, MAC_1, 6) == 0){
        memcpy(broadcastAddress, MAC_2, 6);
    }
    else if (memcmp(own_mac_address, MAC_2, 6) == 0){
        memcpy(broadcastAddress, MAC_1, 6);
    }
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){  
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int lastRecvTime;
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){ //interpret received data here
    memcpy(&espnowDataRecv, incomingData, sizeof(espnowDataRecv));
    otherBotExists = true;
    lastRecvTime = millis();
    // DEBUG(espnowDataRecv.isPresent);
    // DEBUG(espnowDataRecv.inField);
    // DEBUG(espnowDataRecv.xpos);
    // DEBUG(espnowDataRecv.ypos);
    // DEBUG(espnowDataRecv.def);
    // DEBUG(espnowDataRecv.hasBall);
   // Serial.println(espnowDataRecv.isPresent);
}

void set_up_esp_now(){
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }


    //callback functions for sending and receiving
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
}

void sendData(){ //send data here
    //Define what values to send
    espnowData.isPresent = 2;
    espnowData.def = isDefender ? true : false;
    espnowData.xpos = self_x;
    espnowData.ypos = self_y;
    espnowData.hasBall = ballCap ? true : false;
    espnowData.inField = isTilted ? false: true;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espnowData, sizeof(espnowData));

    // switch (result) {
    //     case ESP_OK:
    //         Serial.println("✅ ESP-NOW: Data sent successfully.");
    //         break;
    //     case ESP_ERR_ESPNOW_NOT_INIT:
    //         Serial.println("❌ ESP-NOW: Not initialized.");
    //         break;
    //     case ESP_ERR_ESPNOW_ARG:
    //         Serial.println("❌ ESP-NOW: Invalid argument.");
    //         break;
    //     case ESP_ERR_ESPNOW_INTERNAL:
    //         Serial.println("❌ ESP-NOW: Internal error.");
    //         break;
    //     case ESP_ERR_ESPNOW_NO_MEM:
    //         Serial.println("❌ ESP-NOW: Out of memory.");
    //         break;
    //     case ESP_ERR_ESPNOW_NOT_FOUND:
    //         Serial.println("❌ ESP-NOW: Peer not found.");
    //         break;
    //     case ESP_ERR_ESPNOW_IF:
    //         Serial.println("❌ ESP-NOW: Interface error.");
    //         break;
    //     default:
    //         Serial.print("❌ ESP-NOW: Unknown error: ");
    //         Serial.println(result);
    //         break;
    // }
}

int assignDefBuffer = 4;
int defCount = 0;
int atkCount = 0;
void assignDef(){
    // from lowest to highest priority
    if(espnowDataRecv.def == true){  //check if other bot is defending
        defCount++;
        atkCount = 0;
        if(defCount >= assignDefBuffer){
            isDefender = false;
           // defCount = 0;
        }
    }
    else if (espnowDataRecv.def == false){
        atkCount++;
        defCount = 0;
        if(atkCount >= assignDefBuffer){
            isDefender = true;
           // atkCount = 0;
        }
    }    
    if(isTilted){
        isDefender = false;
    }
    if(ballCap && millis() - lastNoBallCap >= DEFENDER_WAIT_TIME){ //check if the bot has the ball 
        isDefender = false;
    }
    if(espnowDataRecv.inField == false || otherBotExists == false){ //check if the other bot is in the field
        isDefender = true;
    }
    //DEBUG(espnowDataRecv.inField);
}

void getTopPlateData(){
    if(Serial2.available()>=PICO_SERIAL_DATA_LEN){
        while(Serial2.available()>=PICO_SERIAL_DATA_LEN && Serial2.peek()!=5) {
            Serial.println("Pico first byte not 5");
            Serial2.read();
        }
        int len = Serial2.readBytes(uartBufferPico, PICO_SERIAL_DATA_LEN);
        // while(Serial2.available()) Serial2.read();
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
            // DEBUG(self_x);
            // DEBUG(self_y);
            // DEBUG(self_heading);
        }
    }
    else setLED(1, 2, strip.Color(0, 15, 15));
}

void getTopCamData(){
    if(Serial1.available()>=CAM_SERIAL_DATA_LEN){
        while(Serial1.available()>=CAM_SERIAL_DATA_LEN && Serial1.peek()!=5) {
            Serial.println("Camera first byte not 5");
            Serial1.read();
        }
        int len = Serial1.readBytes(uartBufferCam, CAM_SERIAL_DATA_LEN);
        // while(Serial1.available()) Serial1.read();
        if(len!=CAM_SERIAL_DATA_LEN || uartBufferCam[0]!=5){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : uartBufferCam) {
                Serial.print(i);
                Serial.print(" ");
            }
        }
        else{
            top_ball_angle = (float)(uartBufferCam[1] + (uartBufferCam[2]<<8)) / 128;
            top_ball_dist = (float)(uartBufferCam[3] + (uartBufferCam[4]<<8)) / 128;
            top_ball_vx = (float)(uartBufferCam[6] + (uartBufferCam[7]<<8)) / 128;
            if(uartBufferCam[5] == 0) top_ball_vx *= -1;
            top_ball_vy = (float)(uartBufferCam[9] + (uartBufferCam[10]<<8)) / 128;
            if(uartBufferCam[8] == 0) top_ball_vy *= -1;

            //top_ball_vy *= -1;
            //top_ball_vx *= -1;
                    
            DEBUG(top_ball_angle);
            DEBUG(top_ball_dist);
            // DEBUG(top_ball_vx);
            // DEBUG(top_ball_vy);

            if(top_ball_angle==0 && top_ball_dist==0) {
                noBall = true;
                setLED(7, 8, strip.Color(0, 0, 15));
            }
            else {
                noBall = false;
                setLED(7, 8, strip.Color(0, 15, 0));
                lastSeenBall = millis();
            }

            float relative_angle = 90 - (top_ball_angle + self_heading); 
            top_relative_ball_x = (top_ball_dist * cosf(RAD(relative_angle))) / 100;
            top_relative_ball_y = (top_ball_dist * sinf(RAD(relative_angle))) / 100;

            ball_vx = top_ball_vx;
            ball_vy = top_ball_vy;
            ball_vx -= self_velocityx;
            ball_vy -= self_velocityy;
            top_absolute_ball_x = top_relative_ball_x + self_x;
            top_absolute_ball_y = top_relative_ball_y + self_y;

            final_ball_dist = top_ball_dist;
            final_absolute_ball_x = top_absolute_ball_x;
            final_absolute_ball_y = top_absolute_ball_y;

            // DEBUG(ball_vx);
            // DEBUG(ball_vy);
            // DEBUG(top_relative_ball_x);
            // DEBUG(top_relative_ball_y);
            DEBUG(top_absolute_ball_x);
            DEBUG(top_absolute_ball_y);
        }
    }
}

void getBottomPlateData(){
    byte num_bytes = Wire.requestFrom(I2C_RCV_PICO_ADDR, I2C_RCV_DATA_LEN);
    if(num_bytes != I2C_RCV_DATA_LEN){
        Serial.print("Received bad data: ");
        setLED(3, 4, strip.Color(15, 15, 0));
        return;
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

    front_ball_x = (float)(rcvBuffer[2] + (rcvBuffer[3]<<8)) / 128;
    if(rcvBuffer[1]==0) front_ball_x *= -1;
    front_ball_y = (float)(rcvBuffer[5] + (rcvBuffer[6]<<8)) / 128;
    if(rcvBuffer[4]==0) front_ball_y *= -1;
    front_ball_vx = (float)(rcvBuffer[8] + (rcvBuffer[9]<<8)) / 128;
    if(rcvBuffer[7]==0) front_ball_vx *= -1;
    front_ball_vy = (float)(rcvBuffer[11] + (rcvBuffer[12]<<8)) / 128;
    if(rcvBuffer[10]==0) front_ball_vy *= -1;
    
    float front_ball_angle = 90 - DEG(atan2(front_ball_y, front_ball_x));
    float front_ball_dist = sqrt(front_ball_x * front_ball_x + front_ball_y * front_ball_y);
    float front_relative_angle = 90 - (front_ball_angle + self_heading); 
    float front_relative_ball_x = (front_ball_dist * cosf(RAD(front_relative_angle))) / 100;
    float front_relative_ball_y = (front_ball_dist * sinf(RAD(front_relative_angle))) / 100;
    
    front_absolute_ball_x = front_relative_ball_x + self_x;
    front_absolute_ball_y = front_relative_ball_y + self_y;

    if(front_ball_x!=0 && front_ball_y!=0){
        // final_ball_dist = front_ball_dist;
        // final_absolute_ball_x = front_absolute_ball_x;
        // final_absolute_ball_y = front_absolute_ball_y;
        setLED(6, 6, strip.Color(0, 15, 0));
    }
    else setLED(6, 6, strip.Color(0, 0, 15));
    DEBUG(front_absolute_ball_x);
    DEBUG(front_absolute_ball_y);
    DEBUG(final_absolute_ball_x);
    DEBUG(final_absolute_ball_y);

    // DEBUG(front_ball_x);
    // DEBUG(front_ball_y);
    // DEBUG(front_ball_vx);
    // DEBUG(front_ball_vy);

    ballCap = (bool) rcvBuffer[LIDAR_GATE_POS];
    // DEBUG(ballCap);
    // if(ballCap) {
    //     pid_rotate.setConfig(0.05, 0, 0);
    //     pid_x.setConfig(1, 0, 0);
    //     pid_y.setConfig(1, 0, 0);
    // }
    // else {
    //     pid_rotate.setConfig(pid_rotate_default[0], pid_rotate_default[1], pid_rotate_default[2]);
    //     pid_x.setConfig(pid_x_default[0], pid_x_default[1], pid_x_default[2]);
    //     pid_y.setConfig(pid_y_default[0], pid_y_default[1], pid_y_default[2]);
    // }

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
        setLED(9, 10, strip.Color(15, 0, 15));
        lastBallCap = millis();
    }
    else if(!noBall && self_y < final_absolute_ball_y && self_y > final_absolute_ball_y - BALLCAP_DISTANCE 
        && abs(top_relative_ball_x) < BALLCAP_WIDTH / 2.0){
            ballCap = true;
            lastBallCap = millis();
            setLED(9, 10, strip.Color(0, 15, 15));
    }
    else if(millis() - lastBallCap < BALLCAP_DURATION){
        ballCap = true;
        setLED(9, 10, strip.Color(15, 15, 15));
    }
    else {
        lastNoBallCap = millis();
        setLED(9, 10, strip.Color(15, 15, 0));
    }
}

void sendI2C(byte (&buffer)[I2C_SEND_DATA_LEN]){
    Wire.beginTransmission(I2C_SEND_PICO_ADDR);
    Wire.write(buffer, I2C_SEND_DATA_LEN);
    Wire.endTransmission();
}

void movement(float target_x, float target_y, float target_rotation){
    target_x = constrain(target_x, 0.20, FIELD_WIDTH - 0.20);
    if(self_x > FIELD_MARGIN_X && self_x < FIELD_WIDTH - FIELD_MARGIN_X) target_y = constrain(target_y, 0.40, FIELD_HEIGHT - 0.40);
    else target_y = constrain(target_y, 0.20, FIELD_HEIGHT - 0.20);

    if(isDefender) target_y = constrain(target_y, 0, DEFENDER_MAX_YPOS);
    float x_dist = target_x - self_x, y_dist = target_y - self_y;
    float total_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    float total_angle = atan2(y_dist, x_dist) + RAD(self_heading) - PI/4; // in radians

    float rotation_dist = self_heading - target_rotation;
    while(rotation_dist > 180) rotation_dist -= 360;
    while(rotation_dist < -180) rotation_dist += 360;

    float shifted_x_dist = total_dist * sinf(total_angle);
    float shifted_y_dist = total_dist * cosf(total_angle);

    if(ballCap){
        max_translation_pid_value = 0.5;
        max_rotation_pid_value = 0.25;
    }
    else {
        max_translation_pid_value = 1;
        max_rotation_pid_value = 1;
    }

    speed_xdir = pid_x.compute(0, shifted_x_dist);
    speed_ydir = pid_y.compute(0, shifted_y_dist);
    rotation = constrain(pid_rotate.compute(0, RAD(rotation_dist)), -max_rotation_pid_value, max_rotation_pid_value);

    float maxPID = max(abs(speed_xdir), abs(speed_ydir));
    if(maxPID > max_translation_pid_value){
        float k = max_translation_pid_value/maxPID;
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

void oscillateAboutPoint(float pointx, float pointy, float oscillationDist){
    float new_x = 0, new_y = pointy;
    if(oscillateState) new_x = pointx - oscillationDist;
    else new_x = pointx + oscillationDist;
    float distToPoint = sqrt((new_x - self_x)*(new_x - self_x) + (new_y - self_y)*(new_y - self_y));
    if(distToPoint <= 0.10) {
        if(!reachTargetOscillation){
            reachTargetOscillation = true;
            reachOscillatePointTime = millis();
        }
    }
    if(reachTargetOscillation && millis() - reachOscillatePointTime >= OSCILLATE_WAIT_TIME){
        oscillateState = !oscillateState;
        reachTargetOscillation = false;
        reachOscillatePointTime = 0;
    }
    movement(new_x, new_y, 0);
}

// void ballTrack(){
//     float absBallAngle = atan2(top_relative_ball_y - self_y, top_relative_ball_x - self_x);
//     float goalToBallAngle = atan2(2.384 - top_relative_ball_y, 0.91 - top_relative_ball_x);
//     float angleToFace = atan2(2.384 - self_y, 0.91 - self_x);
//     if(absBallAngle <= 60 || absBallAngle >= 300){
//         // movement(self_x + top_relative_ball_x, self_y + top_relative_ball_y - 0.06, 90-DEG(angleToFace));
//         movement(self_x + top_relative_ball_x, self_y + top_relative_ball_y - 0.04, 0);
//     }
//     else{
//         float new_x = self_x + top_relative_ball_x + 0.40 * cosf(goalToBallAngle);
//         float new_y = self_y + top_relative_ball_y + 0.40 * sinf(goalToBallAngle);
//         movement(new_x, new_y, 90-DEG(angleToFace));
//         // movement(self_x + top_relative_ball_x, self_y + top_relative_ball_y - 0.40, 90-DEG(angleToFace));
//         // movement(self_x + top_relative_ball_x, self_y + top_relative_ball_y - 0.40, 0);
//     }
//     // movement(self_x + top_relative_ball_x, self_y + top_relative_ball_y - 0.12, 90-DEG(angleToFace));
// }

void ballTrack(){ // CHANGE top to final maybe
    aligned = false;
    initial_change = 0.0;
    initial_magnitude = 0.0;

    float new_x, new_y;
    if(self_y > top_absolute_ball_y) moving_back = true;
    if((moving_back || millis() - last_moving_back > MOVING_BACK_DURATION) && 
        (self_y > top_absolute_ball_y - BALLCAP_DISTANCE / 3.0 || 
        (abs(self_x - top_absolute_ball_x) > BALLCAP_WIDTH / 2.0 + 0.05f && 
        abs(self_x - top_absolute_ball_x) < CLEARANCE_X / 2.0 && 
        self_y > top_absolute_ball_y - CLEARANCE_Y / 2.0))){
            // Serial.println("case1");
            if(top_absolute_ball_x < FIELD_MARGIN + CLEARANCE_X + 0.10f) new_x = top_absolute_ball_x + (CLEARANCE_X / 2.0 + 0.05f);
            else if(top_absolute_ball_x > FIELD_WIDTH - FIELD_MARGIN - CLEARANCE_X - 0.10f) new_x = top_absolute_ball_x - (CLEARANCE_X / 2.0 + 0.05f);
            else if (self_x > top_absolute_ball_x) new_x = top_absolute_ball_x + (CLEARANCE_X / 2.0 + 0.05f);
            else new_x = top_absolute_ball_x - (CLEARANCE_X / 2.0 + 0.05f);
            new_y = (abs(self_x - top_absolute_ball_x) > CLEARANCE_X / 2.0 + 0.03f) ? top_absolute_ball_y - CLEARANCE_Y / 2.0 - 0.10f : self_y;
            moving_back = true;
    }
    else{
        // Serial.println("case2");
        if(moving_back) {
            moving_back = false;
            last_moving_back = millis();
        }
        unsigned long aligning = millis() - last_aligning;
        new_x = top_absolute_ball_x;
        if((aligning > ALIGN_DURATION && aligning < ALIGN_THRESHOLD) || abs(self_x - top_absolute_ball_x) < BALLCAP_WIDTH / 2.0) 
            new_y = fmax(top_absolute_ball_y - BALLCAP_DISTANCE, self_y + 0.03f) ;
        else{
            if (aligning > ALIGN_THRESHOLD) last_aligning = millis(); 
            new_y = top_absolute_ball_y - BALLCAP_DISTANCE;
        }
    }
    // DEBUG(new_x);
    // DEBUG(new_y);
    movement(new_x, new_y, 0);
}

void dribblerBallTrack(){
    float xToBall = final_absolute_ball_x - self_x, yToBall = final_absolute_ball_y - self_y;
    float distToBall = sqrt(xToBall * xToBall + yToBall * yToBall);
    float new_x = self_x + xToBall * (distToBall - BALLCAP_DISTANCE) / distToBall;
    float new_y = self_y + yToBall * (distToBall - BALLCAP_DISTANCE) / distToBall;

    float absBallAngle = atan2(yToBall, xToBall);
    LIM_ANGLE_180(absBallAngle);
    // DEBUG(absBallAngle);

    movement(new_x, new_y, 90-DEG(absBallAngle));
}

void aim(){
    if(!aligned){
        // Serial.println("aim align");
        if(abs(self_x - final_absolute_ball_x) < ALIGNED_THRESHOLD) aligned = true;
        // DEBUG(self_x);
        // DEBUG(top_absolute_ball_x);
        movement(final_absolute_ball_x, self_y, 0);
    }
    else{
        // Serial.println("aim accel");
        float xToGoal = OPP_GOAL_CENTRE_X - self_x, yToGoal = OPP_GOAL_CENTRE_Y - self_y;
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
        // DEBUG(change);
        // DEBUG(new_x);
        // DEBUG(new_y);
        movement(new_x, new_y, DEG(angleToGoal));
    }
    float minAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self_y, OPP_GOAL_LEFT_X - self_x));
    float maxAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self_y, OPP_GOAL_RIGHT_X - self_x));
    LIM_ANGLE_180(minAngleFace);
    LIM_ANGLE_180(maxAngleFace);
    if(minAngleFace > maxAngleFace) std::swap(minAngleFace, maxAngleFace);
    if(ballCap && self_y > 1.62 && (self_heading >= minAngleFace && self_heading <= maxAngleFace)) {
        kicker.kick();
    }
}

void dribblerAim(){
    float angleToFace = atan2(OPP_GOAL_CENTRE_Y - self_y, OPP_GOAL_CENTRE_X - self_x);
    movement(OPP_GOAL_MIDDLE_X, OPP_GOAL_MIDDLE_Y, 90-DEG(angleToFace));
    float minAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self_y, OPP_GOAL_LEFT_X - self_x));
    float maxAngleFace = 90-DEG(atan2(OPP_GOAL_Y - self_y, OPP_GOAL_RIGHT_X - self_x));
    LIM_ANGLE_180(minAngleFace);
    LIM_ANGLE_180(maxAngleFace);
    if(minAngleFace > maxAngleFace) std::swap(minAngleFace, maxAngleFace);
    if(ballCap && self_y > 1.62 && (self_heading >= minAngleFace && self_heading <= maxAngleFace)) {
        kicker.kick();
        dribbler.setSpeed(-1.0);
        lastDribblerRev = millis();
    }
}

void ballHide(){
    if(self_x < FIELD_WIDTH/2)  ballHideState = false; // left side
    else ballHideState = true; // right side
    if(self_y > 1.90) dribblerAim();
    else if(self_x > 0.35 && self_x < FIELD_WIDTH - 0.35){
        // move to side of field
        if(ballHideState) movement(FIELD_WIDTH - 0.25, self_y, 90);
        else movement(0.25, self_y, -90);
    }
    else {
        if(ballHideState) movement(FIELD_WIDTH - 0.25, 2.0, 90);
        else movement(0.25, 2.0, -90);
    }
}

void frontCamTrack(){ 
    //turns bot to ball based on top cam, use if a more accurate front cam measurement is needed
    float approx_future_x = top_absolute_ball_x + ball_vx * 0.2;
    float approx_future_y = top_absolute_ball_y + ball_vy * 0.2;
    movement(self_x, self_y, atan2(approx_future_x, approx_future_y));
}

void lookAhead(){
    float v = 0.75;
    float latency = 0;
    bool validt = false;
    bool useFrontCam = false;
    bool lookAheadConfirm = false;
    float t;
    float LAball_x, LAball_y, LAball_vx, LAball_vy;

    // if((abs(self_x - current_target_x) >= 0.2 && abs (self_y - current_target_y) >= 0.2) && (current_target_x != 999) && (current_target_y != 99)){ //moving to target
         //lookingAhead = true;
    // }
    // else{ //arrived at target
         //lookingAhead = false;
    // }
    lookingAhead = true;
    if(useFrontCam){
        frontCamTrack();
        LAball_x = front_absolute_ball_x + ball_vx * latency;
        LAball_y = front_absolute_ball_y + ball_vy * latency;
        LAball_vx = front_ball_vx;
        LAball_vy = front_ball_vy;
    }
    else{
        LAball_x = top_absolute_ball_x + ball_vx * latency;
        LAball_y = top_absolute_ball_y + ball_vy * latency;
        LAball_vx = ball_vx;
        LAball_vy = ball_vy;
        LAball_vx -= self_velocityx;
        LAball_vy -= self_velocityy;
        // DEBUG(LAball_vx);
    }

    float C = LAball_x*LAball_x + LAball_y*LAball_y;
    float B = 2*(LAball_x*ball_vx + LAball_y*LAball_vy);
    float A = LAball_vx*LAball_vx + LAball_vy*LAball_vy - v*v;
    validt = false;

    if (abs(A) > pow(10, -8) && (B*B - 4*A*C) >= 0){ //we get two solutions for time, so we want to find the minimum time that is not negative
        float t1 = (-1*B - pow((B*B - 4*A*C), 0.5))/(2*A); 
        float t2 = (-1*B + pow((B*B - 4*A*C), 0.5))/(2*A); 
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
        lookAheadConfirm = true;
    }

    if(!validt){ //ball is too fast
        // float theta_invalid_t;
        // if (abs(LAball_vx) > pow(10, -8)){ 
        //     theta_invalid_t = atan2(LAball_vy,LAball_vx);}
        // else{theta_invalid_t = 3.1415/2;}
        // LAball_vx = 0.9*v*cos(theta_invalid_t); // if magnitude of ball's velocity is less than bot's velocity, it should be interceptable regardless of direction
        // LAball_vy = 0.9*v*sin(theta_invalid_t);
        lookingAhead = false;
        //sendI2C(zeroBuffer);
        dribblerBallTrack();
        lookAheadConfirm = false;
    }  
    

    //assume front is facing towards positive y
    float targetballposx = LAball_x + LAball_vx*t;
    float targetballposy = LAball_y + LAball_vy*t;
    //float targetheadinglookahead = atan2(targetballposx,targetballposy) * (180/3.1415) + 90; 
    // targetballposx = (targetballposx + self_x);
    // targetballposy = (targetballposx + self_y);

    // DEBUG(self_x);
    // DEBUG(self_y);
    // DEBUG(LAball_x);
    // DEBUG(LAball_y);
    // DEBUG(LAball_vx);
    // DEBUG(LAball_vy);
    // DEBUG(t);
    // DEBUG(targetballposy);
    // DEBUG(targetballposx);

    if (lookAheadConfirm){
        float xToBall = top_absolute_ball_x - self_x, yToBall = top_absolute_ball_y - self_y;
        float absBallAngle = atan2(yToBall, xToBall);
        movement(targetballposx, targetballposy, 90-DEG(absBallAngle));
        current_target_x = targetballposx;
        current_target_y = targetballposy;
        Serial.println("Moving to new target");
    }
}

//velocity of robot with moving average

void updateSelfVelocityEWMA(float current_self_x, float current_self_y) {
    unsigned long now = millis();
    float dt = (now - last_vel_time) / 1000.0f; 
    if (dt < 1e-6f) {
        return;
    }

    float inst_vx = (current_self_x - last_self_x) / dt;  
    float inst_vy = (current_self_y - last_self_y) / dt;  

    // Exponential Weighted Moving Average update, beta parameter used = 0.8
    self_velocityx = 0.2f * inst_vx + (0.8f) * self_velocityx;
    self_velocityy = 0.2f * inst_vy + (0.8f) * self_velocityy;

    // Save current data for next iteration
    last_self_x = current_self_x;
    last_self_y = current_self_y;
    last_vel_time   = now;
}

void defend(){
    float leftAngle = atan2(SELF_GOAL_Y - final_absolute_ball_y, SELF_GOAL_LEFT_X - final_absolute_ball_x);
    float rightAngle = atan2(SELF_GOAL_Y - final_absolute_ball_y, SELF_GOAL_RIGHT_X - final_absolute_ball_x);
    float angleDiff = rightAngle - leftAngle;
    LIM_ANGLE_180(angleDiff);
    if(angleDiff < 0){
        float tempAngle = leftAngle;
        leftAngle = rightAngle;
        rightAngle = tempAngle;
        angleDiff = -angleDiff;
    }
    float midAngle = leftAngle + (angleDiff / 2);
    float sinHalfAngle = sinf(angleDiff/2);
    float new_x, new_y, newDistToBall;
    // if(fabs(sinHalfAngle) , 1e-6f) newDistToBall = 9999.0f;
    // else newDistToBall = BOT_RADIUS_M / sinHalfAngle;
    newDistToBall = BOT_RADIUS_M / sinHalfAngle;
    new_x = final_absolute_ball_x + newDistToBall * cosf(midAngle);
    new_y = final_absolute_ball_y + newDistToBall * sinf(midAngle);
    if(new_y > 0.80){
        float denom = (new_x - 0.91f);
        float slope = (new_y - 0.12f)/ (denom);
        new_y = 0.80f;
        float dydefend = (new_y - 0.12f);
        new_x = 0.91f + (dydefend / slope);
    }
    // DEBUG(new_x);
    // DEBUG(new_y);
    movement(new_x, new_y, 0);
}

//// ** LOOPS ** ////

void setup(){
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    Serial2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(PAUSE_SW1, INPUT);
    pinMode(PAUSE_SW2, INPUT);
    pinMode(STATE_SW, INPUT);
    analogSetAttenuation(ADC_11db);

    esp_task_wdt_init(1, true); // timeout in seconds
    enableLoopWDT();

    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    for (int i=0; i<I2C_SEND_DATA_LEN; i++) zeroBuffer[i] = 0;

    dribblerMD.init();
    dribblerMD.setMode();

    readMacAddress();
    set_up_esp_now();

    if(espnowDataRecv.isPresent == 2) isDefender = false;
    else isDefender = true;

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

}

int esp_last_send;
int LA_ball_seen;
float prev_ball_vx, prev_ball_vy;
float prev_prev_ball_vx, prev_prev_ball_vy;
void loop(){
    // Serial.println("running main code");
    float curTime = millis();
    //Serial.print("time: ");
    //Serial.println(curTime - lastLoopTime);
    lastLoopTime = millis();
    DEBUG(isDefender);
    if(millis() - lastLED >= BLINK_TIME){
        esp_led_state = !esp_led_state;
        lastLED = millis();
    }
    if(esp_led_state) esp_led.setPixelColor(0, esp_led.Color(0, 50, 0));
    else esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
    esp_led.show();

    if(digitalRead(TURN_OFF_SW)==LOW) turnOff = true;
    else turnOff = false;

    //readVoltage();
    checkFault();
    getTopPlateData();
    getTopCamData();
    getBottomPlateData();

    ballCapStatus();

    if(digitalRead(STATE_SW)==HIGH && millis() - lastStateSwap >= STATE_SWAP_TIME){
        codeState++;
        codeState %= TOTAL_STATES;
    }

    if(digitalRead(PAUSE_SW1)==HIGH || digitalRead(PAUSE_SW2)==HIGH){
        isTilted = true;
    }
    
    if (curTime - esp_last_send >= 500){
        sendData();
        esp_last_send = curTime;
    }
    updateSelfVelocityEWMA(self_x, self_y);
    if(curTime - lastRecvTime > 1000){
        otherBotExists = false;
    }
    assignDef();
    sendData();
    if(!isDefender){
        LED_BRIGHTNESS = 255;
    }
    else{
        LED_BRIGHTNESS = 60;
    }
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    // Serial.printf("Own MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
    //           own_mac_address[0], own_mac_address[1], own_mac_address[2],
    //           own_mac_address[3], own_mac_address[4], own_mac_address[5]);
    // Serial.printf("Broadcast: %02x:%02x:%02x:%02x:%02x:%02x\n",
    //           broadcastAddress[0], broadcastAddress[1], broadcastAddress[2],
    //           broadcastAddress[3], broadcastAddress[4], broadcastAddress[5]);
    // Serial.println(espnowDataRecv.isPresent);

    float last_ball_dist = sqrt(last_ball_x * last_ball_x + last_ball_y * last_ball_y);
    if(millis() - lastDribblerRev < 1000) ;
    else if(ballCap || (final_ball_dist>0 && final_ball_dist<=40) || (last_ball_dist>0 && last_ball_dist<=40)) dribbler.setSpeed(1.0);
    else if(noBall) dribbler.setSpeed(0);
    else dribbler.setSpeed(0.5);

    if(codeState==0) setLED(11, 11, strip.Color(0, 15, 0)); // green
    else if(codeState==1) setLED(11, 11, strip.Color(0, 15, 15)); // cyan
    else setLED(11, 11, strip.Color(0, 0, 15)); // blue

    if (isDefender){
        pid_rotate.setConfig(pid_def_rotate_default[0], pid_def_rotate_default[1], pid_def_rotate_default[2]);
        pid_x.setConfig(pid_def_x_default[0], pid_def_x_default[1], pid_def_x_default[2]);
        pid_y.setConfig(pid_def_y_default[0], pid_def_y_default[1], pid_def_y_default[2]);
        if (!ballCap){
            if (noBall && (millis() - lastSeenBall) > LAST_SEEN_BALL_TIME) {
                movement(0.91f, 0.60f, 0);
            }
            // 2) Else if the ball is within the no-chase region near the goal
            else if (final_absolute_ball_x > 0.62f && final_absolute_ball_x < 1.20f &&
                    final_absolute_ball_y < 0.25f)
            {
                movement(0.91f, 0.60f, 0);
            }
            // 3) Else if the ball is behind the robot (y < 1.0f => "behind" threshold)
            else if (final_absolute_ball_y <  0.80f) {
                if(millis() - lastDribblerRev < 1000) ;
                else if(ballCap || (final_ball_dist>0 && final_ball_dist<=40) || 
                    (last_ball_dist>0 && last_ball_dist<=40 && millis() - lastSeenBall <= LAST_SEEN_BALL_TIME)) 
                    dribbler.setSpeed(1.0);
                else if(noBall) dribbler.setSpeed(0);
                else dribbler.setSpeed(0.5);

                if (final_absolute_ball_x > 0.62f && final_absolute_ball_x < 1.20f && final_absolute_ball_y < self_y){
                    pid_rotate.setConfig(0.4, 0, 0);
                    pid_x.setConfig(1.9, 0, 0);
                    pid_y.setConfig(1.9, 0, 0);                
                }
                else {
                    pid_rotate.setConfig(0.5, 0, 0);
                    pid_x.setConfig(2.2, 0, 0);
                    pid_y.setConfig(2.2, 0, 0);  
                }
                if (noBall) {
                    final_absolute_ball_x = last_ball_x;
                    final_absolute_ball_y = last_ball_y;
                    dribblerBallTrack();
                }
                // 3b) If we DO see the ball => track it with the dribbler
                else {
                    dribblerBallTrack();
                }
                pid_rotate.setConfig(pid_def_rotate_default[0], pid_def_rotate_default[1], pid_def_rotate_default[2]);
                pid_x.setConfig(pid_def_x_default[0], pid_def_x_default[1], pid_def_x_default[2]);
                pid_y.setConfig(pid_def_y_default[0], pid_def_y_default[1], pid_def_y_default[2]);
            }
            // 4) Otherwise => geometry-based blocking
            else {
                defend();
            }
        }
        else sendI2C(zeroBuffer);

    }
    else {
        if(millis() - lastDribblerRev < 1000) ;
        else if(ballCap || (final_ball_dist>0 && final_ball_dist<=40) || 
            (last_ball_dist>0 && last_ball_dist<=40 && millis() - lastSeenBall <= LAST_SEEN_BALL_TIME)) 
            dribbler.setSpeed(1.0);
        else if(noBall) dribbler.setSpeed(0);
        else dribbler.setSpeed(0.5);

        pid_rotate.setConfig(pid_att_rotate_default[0], pid_att_rotate_default[1], pid_att_rotate_default[2]);
        pid_x.setConfig(pid_att_x_default[0], pid_att_x_default[1], pid_att_x_default[2]);
        pid_y.setConfig(pid_att_y_default[0], pid_att_y_default[1], pid_att_y_default[2]);

        if(codeState==2){
            if(noBall) oscillateAboutPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0.31);
            else if(ballCap) aim();
            else ballTrack();
        }
        else{
            if(millis() - lastNoBallCap >= SCORING_WAIT_TIME && ballCap){
                if(codeState==0) ballHide();
                else if(codeState==1) dribblerAim();
            }
            else if(ballCap) sendI2C(zeroBuffer);
            else if(noBall && millis() - lastSeenBall <= LAST_SEEN_BALL_TIME && final_absolute_ball_y >= ATTACKER_MIN_BALL_YPOS){
                final_absolute_ball_x = last_ball_x;
                final_absolute_ball_y = last_ball_y;
                dribblerBallTrack();
            }
            else if(!noBall && final_absolute_ball_y >= ATTACKER_MIN_BALL_YPOS){
            // else if(!noBall){
                dribblerBallTrack();
                Serial.println("not my ball bro");
            }
            else oscillateAboutPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0.60); 
            // else movement(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
        }
    }
    if(!noBall) {
        last_ball_x = final_absolute_ball_x;
        last_ball_y = final_absolute_ball_y;
    }
}

/*
#ifdef SECOND_BOT
    
    // 1) If no ball and it's been too long, just stay put
    if (noBall && (millis() - lastSeenBall) > LAST_SEEN_BALL_TIME) {
        movement(0.91f, 0.60f, 0);
    }
    // 2) Else if the ball is within the no-chase region near the goal
    else if (top_absolute_ball_x > 0.62f && top_absolute_ball_x < 1.20f &&
             top_absolute_ball_y < 0.25f)
    {
        movement(0.91f, 0.60f, 0);
    }
    // 3) Else if the ball is behind the robot (y < 1.0f => "behind" threshold)
    else if (top_absolute_ball_y <  0.60f) {
        pid_rotate.setConfig(0.5, 0, 0);
        pid_x.setConfig(2.2, 0, 0);
        pid_y.setConfig(2.2, 0, 0);
        if (noBall) {
            top_absolute_ball_x = last_ball_x;
            top_absolute_ball_y = last_ball_y;
            dribblerBallTrack();
        }
        // 3b) If we DO see the ball => track it with the dribbler
        else {
            dribblerBallTrack();
        }
        pid_rotate.setConfig(pid_rotate_default[0], pid_rotate_default[1], pid_rotate_default[2]);
        pid_x.setConfig(pid_x_default[0], pid_x_default[1], pid_x_default[2]);
        pid_y.setConfig(pid_y_default[0], pid_y_default[1], pid_y_default[2]);
    }
    // 4) Otherwise => geometry-based blocking
    else {
        
        defend();

    }
    #elif defined(LOOK_AHEAD)
    if(noBall) movement(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
    //else if(ballCap) aim();
    else if(ball_vx > 0.10 || ball_vy > 0.10){
        if(lookAheadDelay = false){
            LA_ball_seen = curTime;
            lookAheadDelay = true;
        }
        if(lookAheadDelay == true && LA_ball_seen - curTime >= 100){
            lookAheadDelay = false;
            lookAhead();
        }
        else{
            lookAheadDelay = true;
        }
    }
    #elif defined(NO_DRIBBLER)
    if(noBall) oscillateAboutPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0.31);
    else if(ballCap) aim();
    else ballTrack();
    #else
    if(millis() - lastNoBallCap >= SCORING_WAIT_TIME && ballCap) ballHide();
    else if(ballCap) sendI2C(zeroBuffer);
    else if(noBall && millis() - lastSeenBall <= LAST_SEEN_BALL_TIME){
        top_absolute_ball_x = last_ball_x;
        top_absolute_ball_y = last_ball_y;
        dribblerBallTrack();
    }
    else if(!noBall){
        dribblerBallTrack();
    }
    else oscillateAboutPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0.31);
    #endif
*/