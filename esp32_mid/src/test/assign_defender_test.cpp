#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Kicker.h>
#include <UARTComms.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.print(String(#x) + String(": ")); Serial.println(x, 6); 
#else
#define DEBUG(x) 123;
#endif

// #define SECOND_BOT
//  #define LOOK_AHEAD
// #define NO_DRIBBLER

//// ** DEFINITIONS ** ////

// ESP NeoPixel LED
#define ESP_LED 48
int ESP_BRIGHTNESS = 20;
#define BLINK_TIME 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);
// to check if code is running
bool esp_led_state = true;
float lastLED = 0;

// Switches
// Motor software switch
#define TURN_OFF_SW 38
bool turnOff = false;

// UART Comms with top plate
#define TOP_TX_PIN 16
#define TOP_RX_PIN 17
#define TOP_DATA_LEN 9
byte topBuffer[TOP_DATA_LEN];
UARTComms topUART(TOP_TX_PIN, TOP_RX_PIN, topBuffer, TOP_DATA_LEN, Serial2);

// UART Comms with bottom plate (motor drive RP2040)
#define BOTTOM_TX_PIN 41
#define BOTTOM_RX_PIN 42
#define BOTTOM_DATA_LEN 7
#define I2C_SEND_DATA_LEN 7
byte bottomSendBuffer[BOTTOM_DATA_LEN];
UARTComms bottomUART(BOTTOM_TX_PIN, BOTTOM_RX_PIN, bottomSendBuffer, BOTTOM_DATA_LEN, Serial1);

// I2C Comms with bottom plate (sensor RP2040)
#define BOTTOM_SDA_PIN 40
#define BOTTOM_SCL_PIN 39
#define BOTTOM_I2C_DATA_LEN 18
#define BOTTOM_I2C_ADDRESS 0x08

#define FRONT_CAM_DATA_POS 1
#define LIDAR_GATE_POS 13
#define LINE_DATA_POS 14

byte bottomRcvBuffer[BOTTOM_I2C_DATA_LEN+1];

// I2C Comms wit    h middle plate RP2040
#define MID_SDA_PIN 1
#define MID_SCL_PIN 2
#define MID_I2C_SEND_DATA_LEN 8
#define MID_I2C_RCV_DATA_LEN 8
#define MID_I2C_ADDR 0x09
byte midSendBuffer[MID_I2C_SEND_DATA_LEN];
byte midRcvBuffer[MID_I2C_RCV_DATA_LEN];
byte firstbyte = 5;

// Line Sensors
#define NUM_LINE_MUX 4
float line_status[NUM_LINE_MUX];

// PID
float pid_def_rotate_default[3] = {0.7, 0, 0};
float pid_def_x_default[3] = {3.5, 0, 0};
float pid_def_y_default[3] = {3.5, 0, 0};

float pid_att_rotate_default[3] = {0.3, 0, 0};
float pid_att_x_default[3] = {2.2, 0, 0};
float pid_att_y_default[3] = {2.2, 0, 0};

PID pid_rotate(pid_att_rotate_default[0], pid_att_rotate_default[1], pid_att_rotate_default[2], 1000);
PID pid_x(pid_att_x_default[0], pid_att_x_default[1], pid_att_x_default[2], 1000);
PID pid_y(pid_att_y_default[0], pid_att_y_default[1], pid_att_y_default[2], 1000);
float max_translation_pid_value = 1, max_rotation_pid_value = 1;

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
#define LAST_SEEN_BALL_TIME 200
#define SCORING_WAIT_TIME 500
#define DEFENDER_WAIT_TIME 1500
#define DEFENDER_MAX_YPOS 0.70
#define ATTACKER_MIN_BALL_YPOS 0.70
#define OSCILLATE_WAIT_TIME 2000

// Variables
float self_x = 0, self_y = 0, self_heading = 0;
float ball_angle = 0, ball_dist = 0;
float ball_vx = 0, ball_vy = 0;
float relative_ball_x = 0, relative_ball_y = 0;
float absolute_ball_x = 0, absolute_ball_y = 0;
float last_ball_x = 0, last_ball_y = 0;
bool noBall = false, ballCap = false, topOff = false, isOnLine = false;
float lastLoopTime = 0, lastBallCap = 0, lastNoBallCap = 0, lastSeenBall = millis();
float speed_xdir, speed_ydir, rotation;
bool otherBotExists = false;

// ESP Bluetooth Communication
uint8_t broadcastAddress[6] = {0,0,0,0,0,0}; 
uint8_t own_mac_address[6];
typedef struct struct_message {
    int isPresent;
    float xpos; 
    float ypos;
    bool def;
    bool inField;
    float heading;
    bool hasBall; 
    int botID_comm;
} struct_message;
struct_message espnowData;
struct_message espnowDataRecv;


//// ** FUNCTIONS ** ////

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){  
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
 
}
 
int lastRecvTime;
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){ //interpret received data here
    memcpy(&espnowDataRecv, incomingData, sizeof(espnowDataRecv));
    lastRecvTime = millis();
    // DEBUG(espnowDataRecv.isPresent);
    // DEBUG(espnowDataRecv.inField);
    // DEBUG(espnowDataRecv.xpos);
    // DEBUG(espnowDataRecv.ypos);
    // DEBUG(espnowDataRecv.def);
    // DEBUG(espnowDataRecv.hasBall);
    // Serial.println("received data");
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

int botID = 0;
void readMacAddress(){ //read own mac address and set broadcast address to other bot
    WiFi.mode(WIFI_AP_STA);
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
    const uint8_t MAC_1[6] = {0x28, 0x37, 0x2f, 0x85, 0x9d, 0x54}; // id 1 (yes googly eyes)
    const uint8_t MAC_2[6] = {0xd8, 0x3b, 0xda, 0x7f, 0x6a, 0xf4}; // id 2 (no googly eyes, follows bot 1)
    //const uint8_t MAC_3[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 
    if (memcmp(own_mac_address, MAC_1, 6) == 0){
        memcpy(broadcastAddress, MAC_2, 6);
        botID = 1;
    }
    else if (memcmp(own_mac_address, MAC_2, 6) == 0){
        memcpy(broadcastAddress, MAC_1, 6);
        botID = 2;
    }
    else{
        memcpy(broadcastAddress, MAC_2, 6);
        botID = 3;
    }
    }
}

void sendData(){ //send data here
    //Define what values to send
    espnowData.isPresent = 2;
    espnowData.def = isDefender;
    espnowData.inField = (turnOff || topOff) ? false: true;
    espnowData.xpos = self_x;
    espnowData.ypos = self_y;
    espnowData.heading = self_heading;
    espnowData.hasBall = ballCap ? true : false;
    espnowData.botID_comm = botID;
    
    

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&espnowData, sizeof(espnowData));

    switch (result) {
        case ESP_OK:
            Serial.println("✅ ESP-NOW: Data sent successfully.");
            break;
        case ESP_ERR_ESPNOW_NOT_INIT:
            Serial.println("❌ ESP-NOW: Not initialized.");
            break;
        case ESP_ERR_ESPNOW_ARG:
            Serial.println("❌ ESP-NOW: Invalid argument.");
            break;
        case ESP_ERR_ESPNOW_INTERNAL:
            Serial.println("❌ ESP-NOW: Internal error.");
            break;
        case ESP_ERR_ESPNOW_NO_MEM:
            Serial.println("❌ ESP-NOW: Out of memory.");
            break;
        case ESP_ERR_ESPNOW_NOT_FOUND:
            Serial.println("❌ ESP-NOW: Peer not found.");
            break;
        case ESP_ERR_ESPNOW_IF:
            Serial.println("❌ ESP-NOW: Interface error.");
            break;
        default:
            Serial.print("❌ ESP-NOW: Unknown error: ");
            Serial.println(result);
            break;
    }
}

void sendMidPlateData(){
    Wire.beginTransmission(MID_I2C_ADDR);
    Wire.write(midSendBuffer, MID_I2C_SEND_DATA_LEN);
    Wire.endTransmission();
}


void getMidPlateData(){
    byte num_bytes = Wire.requestFrom(MID_I2C_ADDR, MID_I2C_RCV_DATA_LEN);
    if(num_bytes != MID_I2C_RCV_DATA_LEN){
        // Serial.print("Received bad data: ");
        return;
    }
    // else Serial.print("Received: ");
    for (int i=0; i<MID_I2C_RCV_DATA_LEN; i++) {
        if (Wire.available()) {
            midRcvBuffer[i] = Wire.read();
        }
    }
    if(midRcvBuffer[0]!=firstbyte) {
        // Serial.print("Received bad data");
        return;
    }
    ball_angle = (float)(midRcvBuffer[1] + (midRcvBuffer[2]<<8)) / 128;
    ball_dist = (float)(midRcvBuffer[3] + (midRcvBuffer[4]<<8)) / 128;
    // DEBUG(ball_angle);
    // DEBUG(ball_dist);

    if(ball_angle==0 && ball_dist==0) {
        noBall = true;
    }
    else {
        noBall = false;
        lastSeenBall = millis();
    }
    // DEBUG(ball_angle);
    // DEBUG(ball_dist);

    float relative_angle = 90 - (ball_angle + self_heading); 
    relative_ball_x = (ball_dist * cosf(RAD(relative_angle))) / 100;
    relative_ball_y = (ball_dist * sinf(RAD(relative_angle))) / 100;

    absolute_ball_x = relative_ball_x + self_x;
    absolute_ball_y = relative_ball_y + self_y;
}

void getTopPlateData(){
    bool status = topUART.uartRead((byte)5);
    if(status){
        self_x = (float)(topBuffer[1] + (topBuffer[2]<<8)) / 128;
        self_y = (float)(topBuffer[3] + (topBuffer[4]<<8)) / 128;
        self_heading = (float)(topBuffer[6] + (topBuffer[7]<<8)) / 128;
        if(topBuffer[5]==0) self_heading *= -1;
        if(topBuffer[8]==1) topOff = true;
        else topOff = false;
    }
}

float last_top_absolute_ball_x = 0;
float last_top_absolute_ball_y = 0;
float self_velocityw = 0, self_velocityx = 0, self_velocityy = 0;
float last_self_w = 0, last_self_x = 0, last_self_y = 0;
unsigned long last_vel_time = 0;
float dt_ball;
bool updatedBallV = false;
float inst_ball_vx = 0, inst_ball_vy = 0;
void updateSelfVelocityEWMA(float current_self_w, float current_self_x, float current_self_y) { 
    unsigned long now = micros();
    float dt = (now - last_vel_time) / 1000000.0f; 
    if(!updatedBallV){
        dt_ball += abs((now - last_vel_time) / 1000000.0f);
    }
    else{
        dt_ball = abs((now - last_vel_time) / 1000000.0f);
    }
    // Serial.println(dt, 6);
    if (dt < 1e-6f) {
        return;
    }
    
    if(abs(current_self_w - last_self_w) > 6){ //account for 0 -> 2pi
        if(current_self_w > last_self_w){
            current_self_w = 2*3.1415 - current_self_w;
        }
        else{
            last_self_w = 2*3.1415 - last_self_w;
        }
    }

    float inst_vw = (current_self_w - last_self_w) / dt;
    float inst_vx = (current_self_x - last_self_x) / dt;  
    float inst_vy = (current_self_y - last_self_y) / dt; 
    if((absolute_ball_x == 0 && absolute_ball_y == 0) || (absolute_ball_x != last_top_absolute_ball_x || absolute_ball_y != last_top_absolute_ball_y)) {   
        inst_ball_vx = (absolute_ball_x - last_top_absolute_ball_x) / dt_ball;
        inst_ball_vy = (absolute_ball_y - last_top_absolute_ball_y) / dt_ball;
        updatedBallV = true;
    }
    else{
        updatedBallV = false;
    }
    // DEBUG(absolute_ball_x);
    // DEBUG(absolute_ball_y);
    // DEBUG(updatedBallV);
    // DEBUG(dt_ball);

    // Exponential Weighted Moving Average update, beta parameter used = 0.8
    self_velocityw = 0.2f * inst_vw + (0.8f) * self_velocityw;    
    self_velocityx = 0.2f * inst_vx + (0.8f) * self_velocityx;
    self_velocityy = 0.2f * inst_vy + (0.8f) * self_velocityy;
    if (updatedBallV && !noBall && abs(pow((inst_ball_vx*inst_ball_vx+inst_ball_vy*inst_ball_vy),0.5)) < 4){    
        ball_vx = 0.5f * inst_ball_vx + (0.5f) * ball_vx;
        ball_vy = 0.5f * inst_ball_vy + (0.5f) * ball_vy;
    }
    else if(abs(pow((inst_ball_vx*inst_ball_vx+inst_ball_vy*inst_ball_vy),0.5)) > 4){
        // Serial.println("anomalous data cancelled");
    }
    // DEBUG(noBall);
    // DEBUG(absolute_ball_x);
    // DEBUG(absolute_ball_y);

    // Save current data for next iteration
    last_self_w = current_self_w;
    last_self_x = current_self_x;
    last_self_y = current_self_y;
    last_top_absolute_ball_x = absolute_ball_x;
    last_top_absolute_ball_y = absolute_ball_y;
    last_vel_time = now;
    // DEBUG(self_velocityx);
    // DEBUG(self_velocityy);
    // DEBUG(ball_vx);
    // DEBUG(ball_vy);
}


int assignDefBuffer = 4;
int defCount = 0;
int atkCount = 0;
bool isDefender = false;
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
    if((topOff || turnOff)){
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

void movement(float target_x, float target_y, float target_rotation){
    target_x = constrain(target_x, 0.20, FIELD_WIDTH - 0.20);
    if(self_x > FIELD_MARGIN_X && self_x < FIELD_WIDTH - FIELD_MARGIN_X) target_y = constrain(target_y, 0.40, FIELD_HEIGHT - 0.40);
    else target_y = constrain(target_y, 0.20, FIELD_HEIGHT - 0.20);

    float x_dist = target_x - self_x, y_dist = target_y - self_y;
    float total_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    float total_angle = PI/2 - atan2(y_dist, x_dist) - RAD(self_heading); // in radians

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

    bottomSendBuffer[0] = 5;
    if(turnOff || topOff){
        for (int i=1; i<BOTTOM_DATA_LEN; i++) bottomSendBuffer[i] = 0;
        // DEBUG(turnOff);
        // DEBUG(topOff);
    }
    else{
        bottomSendBuffer[1] = speed_x_sign;
        bottomSendBuffer[2] = rounded_speed_x;
        bottomSendBuffer[3] = speed_y_sign;
        bottomSendBuffer[4] = rounded_speed_y;
        bottomSendBuffer[5] = rotation_sign;
        bottomSendBuffer[6] = rounded_rotation;
    }
    bottomUART.uartWrite();
}

int ballhide_test_x = 0.50;
bool moving_right = true;
void ballHide2(){
    if(botID == 1){
      /*
        if (moving_right){
            movement(self_x + 0.05, self_y, 0);
        }
        else{
            movement(self_x - 0.05, self_y, 0);
        }
        if (self_x >= 1.2){
            moving_right = false;
        }
        if (self_x <= 0.7){
            moving_right = true;
        } */
    }
    else if(botID == 2){
        float target_x = espnowDataRecv.xpos + 0.3*sin(RAD(espnowDataRecv.heading));
        float target_y = espnowDataRecv.ypos + 0.3*cos(RAD(espnowDataRecv.heading));
        float target_heading = 90-DEG(atan2(espnowDataRecv.ypos - self_y, espnowDataRecv.xpos - self_x));
        movement(target_x, target_y, target_heading);
    }
}


float targetballposx = FIELD_WIDTH/2;
float targetballposy = 0.80;
float lastLAtargetx = 0, lastLAtargety = 0, lastLAtargetAngle = 0;
float targetheadinglookahead = 0;
void lookAhead(){
    float v = 1.5;
    float latency = 0.2;
    bool validt = false;
    bool useFrontCam = false;
    bool lookAheadConfirm = false;
    float t;
    float LAball_x, LAball_y, LAball_vx, LAball_vy;

    // else{
    //     LAball_x = top_absolute_ball_x + ball_vx * latency;
    //     LAball_y = top_absolute_ball_y + ball_vy * latency;
    //     LAball_vx = ball_vx;
    //     LAball_vy = ball_vy;
        
    //     LAball_vx += self_velocityx;
    //     LAball_vy += self_velocityy;
    //     // DEBUG(LAball_vx);
    // }
    //look ahead uses relative position and absolute velocity
    LAball_x = relative_ball_x;
    LAball_y = relative_ball_y;
    LAball_vx = ball_vx;
    LAball_vy = ball_vy;
    // LAball_vx -= self_velocityx;
    // LAball_vy -= self_velocityy;


    int lookahead_n = 0;
    while(!lookAheadConfirm && lookahead_n < 5){ 
    lookahead_n++;
    float C = LAball_x*LAball_x + LAball_y*LAball_y;
    float B = 2*(LAball_x*LAball_vx + LAball_y*LAball_vy);
    float A = LAball_vx*LAball_vx + LAball_vy*LAball_vy - v*v;
    lookAheadConfirm = false;

    if (abs(A) > pow(10, -8) && (B*B - 4*A*C) >= 0){ //we get two solutions for time, so we want to find the minimum time that is not negative
        float t1 = (-1*B - pow((B*B - 4*A*C), 0.5))/(2*A); 
        float t2 = (-1*B + pow((B*B - 4*A*C), 0.5))/(2*A); 
        if (t1 >= 0 && t2 >= 0){
            t = min(t1, t2);
            lookAheadConfirm = true;
        }
        else if (t1 >= 0){
            t = t1;
            lookAheadConfirm = true;
        }
        else if (t2 >= 0){
            t = t2;
            lookAheadConfirm = true;
        }  
    }

    if(!lookAheadConfirm){ //ball is too fast
        // float theta_invalid_t;
        // if (abs(LAball_vx) > pow(10, -8)){ 
        //     theta_invalid_t = atan2(LAball_vy,LAball_vx);}
        // else{theta_invalid_t = 3.1415/2;}
        // LAball_vx = 0.9*v*cos(theta_invalid_t); // if magnitude of ball's velocity is less than bot's velocity, it should be interceptable regardless of direction
        // LAball_vy = 0.9*v*sin(theta_invalid_t);
        LAball_vx *= 0.75;
        LAball_vy *= 0.75;
        //sendI2C(zeroBuffer);
        //dribblerBallTrack();
        lookAheadConfirm = false;        
    } 
} 
    t += 0.3;
    targetballposx = LAball_x + LAball_vx*t;
    targetballposy = LAball_y + LAball_vy*t;
    targetballposx += self_x; 
    targetballposy += self_y;
    targetheadinglookahead = atan2(targetballposy,targetballposx);
    // DEBUG(targetballposx);
    // DEBUG(targetballposy);
    // DEBUG(LAball_vx);
    // DEBUG(LAball_vy);
    // DEBUG(self_x);
    // DEBUG(self_y);
    // DEBUG(t);
    // DEBUG(lookAheadConfirm);
 
}

int LA_ball_seen;
float prev_ball_vx[50];
float prev_ball_vy[50];
float prev_prev_ball_vx, prev_prev_ball_vy;
int pbvx_size = 49;
bool moveToGoal = false;
#define pbvx prev_ball_vx
#define pbvy prev_ball_vy
float noBallTimer = 0;
bool checkv(int n, float threshold){ //n = how many previous velocities to check, threshold = requirement for velocities to be consistent
    for(int i = 0; i<n; i++){
        if(abs(pbvx[pbvx_size-n] - pbvx[pbvx_size-(n+1)]) > threshold){
            return false;
        }
        if(abs(pbvy[pbvx_size-n] - pbvy[pbvx_size-(n+1)]) > threshold){
            return false;
        }
    }
    return true;
}
bool checkzero(int n){
    for(int i = 0; i<n; i++){
        if (pbvx[pbvx_size-i] != 0 || pbvy[pbvx_size-i] != 0){
            return false;
        }
    }
    return true;
}

//// ** TESTING ** ////

void moveForward(){
    bottomSendBuffer[0] = 5;
    bottomSendBuffer[1] = 1;
    bottomSendBuffer[2] = 0;
    bottomSendBuffer[3] = 1;
    bottomSendBuffer[4] = 255;
    bottomSendBuffer[5] = 1;
    bottomSendBuffer[6] = 0;
    bottomUART.uartWrite();
}

//// ** LOOPS ** ////

float esp_last_send = millis();
void setup(){
    Serial.begin(115200);

    bottomUART.init();
    topUART.init();

    // pinMode(TURN_OFF_SW, INPUT);
    // pinMode(VOLTAGE_PIN, INPUT);
    // pinMode(PAUSE_SW1, INPUT);
    // pinMode(PAUSE_SW2, INPUT);
    // pinMode(STATE_SW, INPUT);
    analogSetAttenuation(ADC_11db);

    esp_task_wdt_init(2, true); // timeout in seconds
    enableLoopWDT();

    Wire.begin(MID_SDA_PIN, MID_SCL_PIN, 400000);
    Wire1.begin(BOTTOM_SDA_PIN, BOTTOM_SCL_PIN, 400000);

    readMacAddress();
    set_up_esp_now();
    esp_last_send = millis();

    // dribblerMD.init();
    // dribblerMD.setMode();

    // startWebSerial();
    // readMacAddress();
    // set_up_esp_now();

    // if(espnowDataRecv.isPresent == 2) isDefender = false;
    // else isDefender = true;

    // strip.begin();
    // strip.setBrightness(LED_BRIGHTNESS);
    // strip.show();

    for (int i=0; i<pbvx_size; i++){
        pbvx[i] = 0;
        pbvy[i] = 0;
    }

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

}

float lastLookAhead = millis();
float targetballposx_current = 0;
float targetballposy_current = 0;
float rotateAngle_LA = 0;
float weightedX = 0, weightedY = 0;
float posWeight = 1, ballWeight = 0;
#define LOOK_AHEAD_THRESHOLD_T 0
#define LOOK_AHEAD_THRESHOLD_DMIN 0
#define LOOK_AHEAD_THRESHOLD_DMAX 2
#define TARGET_ANGLE_CONST 4
bool rotateBot_LA = true;
bool tooklastball = false;
void loop(){
    // Serial.println("running main code");
    float curTime = millis();
    // Serial.print("time: ");
    // Serial.println(curTime - lastLoopTime);
    lastLoopTime = millis();
    if(millis() - lastLED >= BLINK_TIME){
        esp_led_state = !esp_led_state;
        lastLED = millis();
    }
    if(esp_led_state) esp_led.setPixelColor(0, esp_led.Color(0, 50, 0));
    else esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
    esp_led.show();

    getTopPlateData();
    getMidPlateData();

    assignDef();

    if(noBall && millis() - lastSeenBall <= LAST_SEEN_BALL_TIME){
        absolute_ball_x = last_ball_x;
        absolute_ball_y = last_ball_y;
        // noBall = false;
        tooklastball = true;
    }
    else tooklastball = false;
    if (curTime - esp_last_send >= 100){
        sendData();
        esp_last_send = curTime;
    }
        if(curTime - lastRecvTime > 1000){
        otherBotExists = false;
    }
    for (int i  = 0; i < pbvx_size; i++){ //stores the last 50 values
        pbvx[i] = pbvx[i+1];
        pbvy[i] = pbvy[i+1];
    }
    if (!noBall){
        pbvx[pbvx_size] = ball_vx;
        pbvy[pbvx_size] = ball_vy;
    }
    else{
        pbvx[pbvx_size] = 0;
        pbvy[pbvx_size] = 0;  
    }

    updateSelfVelocityEWMA(RAD(self_heading), self_x, self_y); 

}