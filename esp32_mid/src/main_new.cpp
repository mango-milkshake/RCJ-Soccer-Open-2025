#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Kicker.h>
#include <UARTComms.h>
#include <Data.h>
#include <Bot.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

// #define TESTING

// #define NO_COMMS
#define SINGLE_BOT

// #define SECOND_BOT
//  #define LOOK_AHEAD
// #define NO_DRIBBLER

//// ** DEFINITIONS ** ////


// #define DEF_Y_DEFAULT 0.50

// ESP NeoPixel LED
#define ESP_LED 48
int ESP_BRIGHTNESS = 20;
#define BLINK_TIME 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);
// to check if code is running
bool esp_led_state = true;
float lastLED = 0;

// LEDs on middle plate (small)
#define LEDS_PIN 18
#define LEDS_BRIGHTNESS 50
#define NUM_LEDS 8
Adafruit_NeoPixel leds(NUM_LEDS, LEDS_PIN, NEO_GRB + NEO_KHZ800);

// Switches
// Dribbler software switch
#define DRIB_SW 38

// Motor testing switch
#define MOTOR_TEST_SW 4

// UART Comms with top plate
#define TOP_TX_PIN 16
#define TOP_RX_PIN 17
#define TOP_DATA_LEN 10
byte topBuffer[TOP_DATA_LEN];
UARTComms topUART(TOP_TX_PIN, TOP_RX_PIN, topBuffer, TOP_DATA_LEN, Serial2);

// UART Comms with bottom plate (motor drive RP2040)
#define BOTTOM_TX_PIN 41
#define BOTTOM_RX_PIN 42
#define BOTTOM_DATA_LEN 7
byte bottomSendBuffer[BOTTOM_DATA_LEN];
UARTComms bottomUART(BOTTOM_TX_PIN, BOTTOM_RX_PIN, bottomSendBuffer, BOTTOM_DATA_LEN, Serial1);

// I2C Comms with middle plate RP2040
#define MID_SDA_PIN 1
#define MID_SCL_PIN 2
#define MID_I2C_SEND_DATA_LEN 8
#define MID_I2C_RCV_DATA_LEN 8
#define MID_I2C_ADDR 0x09
byte midSendBuffer[MID_I2C_SEND_DATA_LEN];
byte midRcvBuffer[MID_I2C_RCV_DATA_LEN];
byte firstbyte = 5;

// I2C Comms with bottom plate (sensor RP2040)
#define BOTTOM_SDA_PIN 40
#define BOTTOM_SCL_PIN 39
#define BOTTOM_I2C_DATA_LEN 3
#define BOTTOM_I2C_ADDR 0x08
byte bottomRcvBuffer[BOTTOM_I2C_DATA_LEN];

// Dribbler
#define MOSI_PIN 12
#define MISO_PIN 13
#define SCK_PIN 14
#define CS_PIN 15
#define DRIBBLER_IN1 47
#define DRIBBLER_IN2 48
#define DRIBBLER_NFAULT 21
#define NSLEEP_PIN 6
#define DRVOFF_PIN 7
#define IPROPI_PIN 5
MotorDriver dribblerMD(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN, IPROPI_PIN);
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, drib.maxspeed, 1.0);

// Kicker
#define KICKER_PIN 11
#define KICKER_SW 8
Kicker kicker(KICKER_PIN);

// PID
float pid_def_rotate_default[3] = {8, 0, 0}; // 0.7
float pid_def_x_default[3] = {140, 0, 0}; // 3.5
float pid_def_y_default[3] = {140, 0, 0}; // 3.5

float pid_att_rotate_default[3] = {12, 0, 0};
float pid_att_x_default[3] = {120, 0, 0};
float pid_att_y_default[3] = {120, 0, 0};

PID pid_rotate(pid_att_rotate_default[0], pid_att_rotate_default[1], pid_att_rotate_default[2], 1000);
PID pid_x(pid_att_x_default[0], pid_att_x_default[1], pid_att_x_default[2], 1000);
PID pid_y(pid_att_y_default[0], pid_att_y_default[1], pid_att_y_default[2], 1000);

// Strategies
#define CHANGE_TIME 5000
#define NUM_STRAT_TYPES 3
#define NUM_NO_BALL_STRAT 1
#define NUM_BALL_STRAT 1
#define NUM_SCORE_STRAT 2
#define MAX_NUM_STRATS 2
int stratTypes[NUM_STRAT_TYPES] = {NUM_NO_BALL_STRAT, NUM_BALL_STRAT, NUM_SCORE_STRAT};
int strats[NUM_STRAT_TYPES][MAX_NUM_STRATS] = {{0}, {7, 10}, {6, 8}};

// Variables
float lastLoopTime = 0;
float speed_xdir, speed_ydir, rotation;

// ESP Bluetooth Communication
float esp_last_send = 0;
uint8_t broadcastAddress[6] = {0,0,0,0,0,0}; 
uint8_t own_mac_address[6];
typedef struct struct_message {
    int isPresent;
    float xpos; 
    float ypos;
    float heading;
    bool hasBall; 
    int botID_comm;
    int bh_ready;
    int bh_stage; //aaaaaaaa
    int type;
} struct_message;
struct_message espnowData;
struct_message espnowDataRecv;


// Game logic
#define MAX_DEF_Y 0.80


//// ** FUNCTIONS ** ////
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){  
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
int lastRecvTime;
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){ //interpret received data here
    memcpy(&espnowDataRecv, incomingData, sizeof(espnowDataRecv));
    lastRecvTime = millis();
    comms.bh_stage = espnowDataRecv.bh_stage;
    comms.xpos = espnowDataRecv.xpos;
    comms.ypos = espnowDataRecv.ypos;
    comms.isPresent = espnowDataRecv.isPresent;
    Serial.println("Under onDataRecv");
    DEBUG(espnowDataRecv.bh_stage);
    DEBUG(comms.bh_stage);
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
    const uint8_t MAC_1[6] = {0xd8, 0x3b, 0xda, 0x7c, 0xf3, 0xc8}; // id 1 (defender)
    const uint8_t MAC_2[6] = {0xfc, 0x01, 0x2c, 0x2d, 0xa6, 0x30}; // id 2 (attacker)
    //const uint8_t MAC_3[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 
    if (memcmp(own_mac_address, MAC_1, 6) == 0){
        memcpy(broadcastAddress, MAC_2, 6);
        state.botID = 1;
    }
    else if (memcmp(own_mac_address, MAC_2, 6) == 0){
        memcpy(broadcastAddress, MAC_1, 6);
        state.botID = 2;
    }
    else{
        memcpy(broadcastAddress, MAC_2, 6);
        state.botID = 3;
    }
    }
}

bool ready_to_ballhide = false;
void sendData(){ //send data here
    //Define what values to send
    Serial.println("under SendData:");
    DEBUG(state.ballhide_stage);
    espnowData.isPresent = 2;
    espnowData.xpos = self.x;
    espnowData.ypos = self.y;
    espnowData.heading = self.heading;
    espnowData.hasBall = ball.ballCap > 0 ? true : false;
    espnowData.botID_comm = state.botID;
    espnowData.bh_ready = state.ready_to_shoot;
    espnowData.bh_stage = state.ballhide_stage;
    espnowData.type = state.botType;
    DEBUG(state.ballhide_stage);

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
    int rounded_coord_x = floor(self.x * 128);
    int rounded_coord_y = floor(self.y * 128);
    int uart_heading = floor(abs(self.heading) * 128);

    midSendBuffer[0] = 5;
    midSendBuffer[1] = rounded_coord_x & 0xFF;
    midSendBuffer[2] = (rounded_coord_x >> 8) & 0xFF;
    midSendBuffer[3] = rounded_coord_y & 0xFF;
    midSendBuffer[4] = (rounded_coord_y >> 8) & 0xFF;

    if(copysign(1, self.heading)==1) midSendBuffer[5] = 1;
    else midSendBuffer[5] = 0;
    midSendBuffer[6] = uart_heading & 0xFF;
    midSendBuffer[7] = (uart_heading >> 8) & 0xFF;
    
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
        // Serial.print("Received mid bad data");
        return;
    }
    ball.angle = (float)(midRcvBuffer[1] + (midRcvBuffer[2]<<8)) / 128;
    ball.dist = (float)(midRcvBuffer[3] + (midRcvBuffer[4]<<8)) / 128;

    if(midRcvBuffer[5] == 1) {
        goal.frontPathClear = true;
        // leds.setPixelColor(1, leds.Color(0, 15, 0));
    }
    else {
        goal.frontPathClear = false;
        // leds.setPixelColor(1, leds.Color(0, 0, 15));
    }
    // leds.show();
    goal.open_rows_start = midRcvBuffer[6];
    goal.open_rows_end = midRcvBuffer[7];

    if(ball.angle==0 && ball.dist==0) {
        ball.noBall = true;
    }
    else {
        ball.noBall = false;
        ball.lastSeenBall = millis();
    }
    // DEBUG(ball.angle);
    // DEBUG(ball.dist);

    float relative_angle = 90 - (ball.angle + self.heading); 
    ball.relative_x = (ball.dist * cosf(RAD(relative_angle))) / 100;
    ball.relative_y = (ball.dist * sinf(RAD(relative_angle))) / 100;

    ball.absolute_x = ball.relative_x + self.x;
    ball.absolute_y = ball.relative_y + self.y;

    // DEBUG(ball.absolute_x);
    // DEBUG(ball.absolute_y);
}

void getTopPlateData(){
    bool status = topUART.uartRead((byte)5);
    if(status){
        self.x = (float)(topBuffer[1] + (topBuffer[2]<<8)) / 128;
        self.y = (float)(topBuffer[3] + (topBuffer[4]<<8)) / 128;
        self.heading = (float)(topBuffer[6] + (topBuffer[7]<<8)) / 128;
        if(topBuffer[5]==0) self.heading *= -1;
        if(topBuffer[8]==1) switches.topOff = true;
        else switches.topOff = false;
        state.topStrat = topBuffer[9];
    }
}

void getBottomPlateData(){
    byte num_bytes = Wire1.requestFrom(BOTTOM_I2C_ADDR, BOTTOM_I2C_DATA_LEN);
    if(num_bytes != BOTTOM_I2C_DATA_LEN){
        // Serial.print("Received bottom bad data: ");
        return;
    }
    // else Serial.print("Received: ");
    for (int i=0; i<BOTTOM_I2C_DATA_LEN; i++) {
        if (Wire1.available()) {
            bottomRcvBuffer[i] = Wire1.read();
        }
    }
    if(bottomRcvBuffer[0]!=firstbyte) {
        // Serial.print("Received bad data");
        return;
    }   

    ball.ballCap = bottomRcvBuffer[1];
    if(ball.ballCap > 0) ball.lastBallCap = millis();
    if(ball.ballCap == 0 && millis() - ball.lastBallCap <= BALLCAP_DURATION) ball.ballCap = 2;
    if(ball.ballCap == 0) ball.lastNoBallCap = millis();
    
    self.line_status = bottomRcvBuffer[2];
    if(self.line_status > 0) self.onLine = true;
    else self.onLine = false;

    DEBUG(ball.ballCap);
    // DEBUG(self.onLine);
}

void sendMotorData(){ // fill bottomSendBuffer with desired data before calling this function
    bottomUART.uartWrite();
}

void movement(float target_x, float target_y, float target_rotation){
    target_x = constrain(target_x, 0.20, FIELD_WIDTH - 0.20);
    if(self.x > FIELD_MARGIN_X && self.x < FIELD_WIDTH - FIELD_MARGIN_X) target_y = constrain(target_y, 0.40, FIELD_HEIGHT - 0.40);
    else target_y = constrain(target_y, 0.20, FIELD_HEIGHT - 0.20);

    float x_dist = target_x - self.x, y_dist = target_y - self.y;
    float total_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    // float total_angle = PI/2 - atan2(y_dist, x_dist) - RAD(self.heading); // in radians
    float total_angle = atan2(y_dist, x_dist) + RAD(self.heading) - PI/4; // in radians

    float rotation_dist = self.heading - target_rotation;
    while(rotation_dist > 180) rotation_dist -= 360;
    while(rotation_dist < -180) rotation_dist += 360;

    float shifted_x_dist = total_dist * sinf(total_angle);
    float shifted_y_dist = total_dist * cosf(total_angle);

    speed_xdir = pid_x.compute(0, shifted_x_dist);
    speed_ydir = pid_y.compute(0, shifted_y_dist);
    rotation = constrain(pid_rotate.compute(0, RAD(rotation_dist)), move.min_rotation, move.max_rotation);

    float maxPID = max(abs(speed_xdir), abs(speed_ydir));
    if(maxPID > move.max_translation){
        float k = move.max_translation/maxPID;
        speed_xdir *= k;
        speed_ydir *= k;
    }

    if(abs(speed_xdir) > 2){
        speed_xdir += copysign(move.x_offset, speed_xdir);
    }
    if(abs(speed_ydir) > 2){
        speed_ydir += copysign(move.y_offset, speed_ydir);
    }
    if(abs(rotation) > 2){
        rotation += copysign(move.rotation_offset, rotation);
    }

    uint8_t rotation_sign, speed_x_sign, speed_y_sign;
    if(copysign(1, rotation)==1) rotation_sign = 1;
    else rotation_sign = 0;
    if(copysign(1, speed_xdir)==1) speed_x_sign = 1;
    else speed_x_sign = 0;
    if(copysign(1, speed_ydir)==1) speed_y_sign = 1;
    else speed_y_sign = 0;
    uint8_t rounded_rotation = ((int)floor(abs(rotation))) & 0xFF;
    uint8_t rounded_speed_x = ((int)floor(abs(speed_xdir))) & 0xFF;
    uint8_t rounded_speed_y = ((int)floor(abs(speed_ydir))) & 0xFF;

    bottomSendBuffer[0] = 5;
    if(switches.turnOff || switches.topOff){
        for (int i=1; i<BOTTOM_DATA_LEN; i++) bottomSendBuffer[i] = 0;
    }
    else{
        bottomSendBuffer[1] = speed_x_sign;
        bottomSendBuffer[2] = rounded_speed_x;
        bottomSendBuffer[3] = speed_y_sign;
        bottomSendBuffer[4] = rounded_speed_y;
        bottomSendBuffer[5] = rotation_sign;
        bottomSendBuffer[6] = rounded_rotation;
    }
    sendMotorData();
}

void stop_motors(){
    bottomSendBuffer[0] = 5;
    for (int i=1; i<BOTTOM_DATA_LEN; i++) bottomSendBuffer[i] = 0;
    sendMotorData();
}

int assignTypeBuffer = 4;
int defCount = 0;
int atkCount = 0;
#define DEFENDER_WAIT_TIME 0
int ballhide_strat = 1;
void assignType(){
    // from lowest to highest priority
    if(state.botID == 1){
        state.botType = 1;
    }
    else if(state.botID == 2){
        state.botType = 2;
    }
    if(espnowDataRecv.type == 1){  //check if other bot is defending
        atkCount++;
        defCount = 0;
        if(atkCount >= assignTypeBuffer){
            state.botType = 2;
           // defCount = 0;
        }
    }
    else if (espnowDataRecv.type == 2){ //check if other bot is attacking but not ballhiding
        defCount++;
        atkCount = 0;
        if(defCount >= assignTypeBuffer){
            state.botType = 1;
           // atkCount = 0;
        }
    }  
    if(ballhide_strat == 1){
        #ifdef SINGLE_BOT // IMPT TESTTTT
        if(ball.ballCap > 0) state.botType = 3;
        else if(espnowDataRecv.type == 3) state.botType = 1; // go to defend if other bot is ballcapped
        #else
        if(ball.ballCap == 0 && espnowDataRecv.hasBall == false) state.botType = state.botID;
        else if(ball.ballCap > 0){ // switch states but just dont move yet
        // if(ball.ballCap > 0 && millis() - ball.lastNoBallCap >= DEFENDER_WAIT_TIME){ //check if the bot has the ball 
            state.botType = 3; //switch to scoring
        } 
        else if (espnowDataRecv.type == 3){ //check if other bot is scoring
            state.botType = 3; //switch to 3 to help with ballhide
        }
        #endif
    }
    else if (ballhide_strat == 2){
        if(espnowDataRecv.type == 4){ //grr
            state.botType = 5; //switch to leading
            times.lastBallhideTime = millis();                

        } 
        else if (ball.ballCap > 0){ //check if other bot is leading
            state.botType = 4; //switch to following to help with ballhide
            times.lastBallhideTime = millis();                
        }
        else{
            if(abs(times.curTime - times.lastBallhideTime) > 2000){
                state.ballhide_stage = 0; //reset for next ballhide
            }
        } 
    }     
    if(switches.turnOff || switches.topOff){ //check if bot is off
        state.botType = 0;
        // DEBUG(switches.turnOff);
        // DEBUG(switches.topOff);
    }
    
    //DEBUG(espnowDataRecv.inField);
}
//// ** TESTING ** ////

void motorTest(){
    bottomSendBuffer[0] = 5;
    bottomSendBuffer[1] = 1;
    bottomSendBuffer[2] = 0;
    bottomSendBuffer[3] = 1;
    bottomSendBuffer[4] = 0;
    bottomSendBuffer[5] = 1;
    bottomSendBuffer[6] = 30;
    sendMotorData();
}

void moveForward(){
    bottomSendBuffer[0] = 5;
    bottomSendBuffer[1] = 1;
    bottomSendBuffer[2] = 0;
    bottomSendBuffer[3] = 1;
    bottomSendBuffer[4] = 30;
    bottomSendBuffer[5] = 1;
    bottomSendBuffer[6] = 0;
    sendMotorData();
}

//// ** LOOPS ** ////

void setup(){
    delay(5000); // wait to plug in mid plate power
    Serial.begin(115200);

    bottomUART.init();
    topUART.init();

    pinMode(DRIB_SW, INPUT);
    pinMode(MOTOR_TEST_SW, INPUT);
    // pinMode(VOLTAGE_PIN, INPUT);
    // pinMode(PAUSE_SW1, INPUT);
    // pinMode(PAUSE_SW2, INPUT);
    // pinMode(STATE_SW, INPUT);
    analogSetAttenuation(ADC_11db);

    esp_task_wdt_init(2, true); // timeout in seconds
    enableLoopWDT();

    Wire.begin(MID_SDA_PIN, MID_SCL_PIN, 400000);
    Wire1.begin(BOTTOM_SDA_PIN, BOTTOM_SCL_PIN, 400000);

    dribblerMD.init();
    dribblerMD.setMode();

    pinMode(KICKER_SW, INPUT);

    // startWebSerial();
    readMacAddress();
    set_up_esp_now();
    esp_last_send = millis();

    // if(espnowDataRecv.isPresent == 2) isDefender = false;
    // else isDefender = true;

    // strip.begin();
    // strip.setBrightness(LED_BRIGHTNESS);
    // strip.show();

    state.lastType = 0;
    state.curType = 0;
    state.strategies = static_cast<State::Strategies>(0);
    state.curStratIdx = 0;
    state.lastChange = millis();


    leds.begin();
    leds.setBrightness(LEDS_BRIGHTNESS);
    leds.show();

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

}

void loop(){
    // Serial.println("running main code");
    times.curTime = millis();
    // Serial.print("time: ");
    // Serial.println(times.curTime - lastLoopTime);
    // lastLoopTime = millis();

    // if(millis() - lastLED >= BLINK_TIME){
    //     esp_led_state = !esp_led_state;
    //     lastLED = millis();
    // }
    // if(esp_led_state) esp_led.setPixelColor(0, esp_led.Color(0, 50, 0));
    // else esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
    // esp_led.show();

    getTopPlateData();
    getMidPlateData();
    // sendMidPlateData();
    getBottomPlateData();

    if(digitalRead(MOTOR_TEST_SW)==HIGH){
        if(!switches.motorTest){
            times.motorTestPressed = millis();
            switches.motorTest = true;
        }
        if(millis() - times.motorTestPressed >= times.motorTestWait) motorTest();
        else stop_motors();
        return;
    }
    else switches.motorTest = false;

    if(digitalRead(DRIB_SW)==HIGH) switches.dribOff = false;
    else switches.dribOff = true;

    move.kick = false;
    move.dont_move = false;
    // DEBUG(self.x);
    // DEBUG(self.y);

    // temp ball cap via cam
    // if(!ball.noBall && (ball.angle > 345 || ball.angle < 23) && ball.dist <= 10 /*in cm*/) {
    //     ball.ballCap = 2;
    //     ball.lastBallCap = millis();
    // }
    // else if(millis() - ball.lastBallCap <= 1000) ball.ballCap = 2;
    // else {
    //     ball.ballCap = 0; 
    //     ball.lastNoBallCap = millis();
    // }

    if(ball.noBall && millis() - ball.lastSeenBall <= LAST_SEEN_BALL_TIME){ //using memory
        ball.absolute_x = ball.last_x;
        ball.absolute_y = ball.last_y;
        ball.dist = ball.last_dist;
        ball.noBall = false;
        ball.tooklastball = true;
    }
    else ball.tooklastball = false;

    assignType();

    #ifdef NO_COMMS
    state.botType = state.botID + 10;
    #endif

    if(ball.ballCap != 0 && (state.botType == 1 || state.botType == 11)){
        pid_rotate.setConfig(pid_def_rotate_default[0], pid_def_rotate_default[1], pid_def_rotate_default[2]);
        pid_x.setConfig(pid_def_x_default[0], pid_def_x_default[1], pid_def_x_default[2]);
        pid_y.setConfig(pid_def_y_default[0], pid_def_y_default[1], pid_def_y_default[2]);
    }
    else{
        pid_rotate.setConfig(pid_att_rotate_default[0], pid_att_rotate_default[1], pid_att_rotate_default[2]);
        pid_x.setConfig(pid_att_x_default[0], pid_att_x_default[1], pid_att_x_default[2]);
        pid_y.setConfig(pid_att_y_default[0], pid_att_y_default[1], pid_att_y_default[2]);
    }

    // decide strategy type
    switch (state.botType){
    case 1:  //defender       
        // state.curType = 1;
        // state.curStratIdx = 0; //Defend
        state.strategies = static_cast<State::Strategies>(7);
        Serial.println("defend"); 
        break;
    case 2: //attacker
        if(ball.noBall && millis() - ball.lastSeenBall > 1500){
            // state.curType = 0;
            // state.curStratIdx = 0;
            state.strategies = static_cast<State::Strategies>(2);
        }
        else{ 
            // state.curType = 1;
            // state.curStratIdx = 1; //lookahead
            state.strategies = static_cast<State::Strategies>(10);
            Serial.println("lookahead"); // here
        }
        break;
    case 3: //scoring
        #ifdef SINGLE_BOT
        state.strategies = static_cast<State::Strategies>(9); // TEST IMPT
        #else
        if(state.ready_to_shoot && (espnowDataRecv.bh_ready == true || espnowDataRecv.type != 3 || abs(times.bhScore_timeout - times.curTime) > 5000)){ 
            //shoot if other bot ready or other bot not scoring or timeout reached
            // state.curType = 2;
            // state.curStratIdx = 0; //score
            // Serial.println("score");
            state.strategies = static_cast<State::Strategies>(6);
        }
        else{
            // state.curType = 2;
            // state.curStratIdx = 1; //ballhide
            state.strategies = static_cast<State::Strategies>(8);
            Serial.println("ballhide");            
        }
        #endif
        break;
    case 4: //leading
        if(state.ready_to_shoot){
            state.strategies = static_cast<State::Strategies>(6);
        }
        else{
            state.strategies = static_cast<State::Strategies>(11);
        }
        break;
    case 5: //following
        state.strategies = static_cast<State::Strategies>(12);
        break;
    default:
        state.curType = 0;
        state.curStratIdx = 0;
        // Serial.println("out");
        state.strategies = static_cast<State::Strategies>(0);
        break;
    }
    DEBUG(state.botType);
    DEBUG(state.strategies);
    DEBUG(espnowDataRecv.type);
    // DEBUG(state.botID);
    // DEBUG(espnowDataRecv.type); // HEREE
    // DEBUG(strats[state.curType][state.curStratIdx]);
    
    state.ready_to_shoot = false; // set back for rechecking (after type is assigned)

    if(ball.ballCap) {
        // state.curType = 2; // score
        leds.setPixelColor(1, esp_led.Color(15, 15, 15));
    }
    // else if(ball.noBall && millis() - ball.lastSeenBall <= 1000){
    //     Serial.println("using last ball pos");
    //     ball.absolute_x = ball.last_x;
    //     ball.absolute_y = ball.last_y;
    //     ball.dist = ball.last_dist;
    //     // state.curType = 1;
    //     leds.setPixelColor(1, esp_led.Color(50, 50, 50));
    // }
    else if(ball.noBall) {
        // state.curType = 0; // no ball
        leds.setPixelColor(1, esp_led.Color(0, 0, 15));
    }
    else {
        // state.curType = 1; // ball track
        leds.setPixelColor(1, esp_led.Color(0, 15, 0));
    }
    leds.show();

    // if(ball.ballCap > 0 || ball.dist <= 20 || state.botType == 3 || state.botType == 4 || state.botType == 5 ){
    if(ball.ballCap > 0 || ball.dist <= 20 || state.botType == 4 || state.botType == 5 ){
        move.max_translation = move.translation_ballcap, move.min_translation = -move.translation_ballcap;
        // rotation positive is counterclockwise
        if(ball.ballCap == 1){
            move.min_rotation = -move.rotation_ballcap;
        }
        else move.min_rotation = -move.rotation_lowered;
        if(ball.ballCap == 3){
            move.max_rotation = move.rotation_ballcap;
        }
        else move.max_rotation = move.rotation_lowered;
    }
    else{
        move.max_translation = move.translation_default, move.min_translation = -move.translation_default;
        move.max_rotation = move.rotation_default, move.min_rotation = -move.rotation_default;
    } 

    // decide specific strategy
    // if(state.curType != state.lastType){
    //     state.curStratIdx = 0;
    //     state.lastChange = millis();
    // }
    // else if(millis() - state.lastChange > CHANGE_TIME){
    //     state.curStratIdx++;
    //     state.curStratIdx %= stratTypes[state.curType];
    //     state.lastChange = millis();
    // }
    // state.strategies = static_cast<State::Strategies>(strats[state.curType][state.curStratIdx]);

    // dribbler setting speed (may be changed again in strategies)
    if(switches.turnOff || switches.topOff || switches.dribOff) drib.desired = 0;
    else if(ball.ballCap > 0) drib.desired = drib.maxspeed;
    else if(ball.dist <= 20) drib.desired = drib.track_speed;
    else if(ball.dist <= 40) drib.desired = drib.track_speed/2;
    else if(ball.noBall) drib.desired = 0;

    #ifdef TESTING
    state.strategies = State::Strategies::ATTACK_MODE2;
    #endif

    if(state.botType == 11) state.strategies = State::Strategies::DEFEND_BASIC;
    else if(state.botType == 12) state.strategies = State::Strategies::ATTACK_BASIC;

    // carry out the strategy
    switch (state.strategies){
        case State::Strategies::NONE:
            // any testing code
            bot.moveToPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
            break;

        case State::Strategies::MOVE_TO_POINT:
            // Serial.println("move to centre");
            bot.moveToPoint(FIELD_WIDTH/2, 0.80 /* FIELD_HEIGHT/2 */, 0);
            break;
        
        case State::Strategies::OSCILLATE_ABOUT_POINT:
            // Serial.println("oscillate");
            bot.oscillateAboutPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0.80);
            break;

        case State::Strategies::NO_DRIBBLER_BALL_TRACK:
            // Serial.println("no dribbler ball track");
            bot.ballTrack();
            break;
        
        case State::Strategies::DRIBBLER_BALL_TRACK:
            // Serial.println("dribbler ball track");
            bot.dribblerBallTrack();
            break;
        
        case State::Strategies::NO_DRIBBLER_SCORE:
            // Serial.println("no dribbler score");
            bot.aim();
            break;
        
        case State::Strategies::DRIBBLER_SCORE:
            // Serial.println("dribbler score");
            state.ready_to_shoot = true;
            if(ball.ballCap && (millis() - ball.lastNoBallCap < ball.ballCapTime || drib.speed < drib.reach_speed)){
                // leds.setPixelColor(7, leds.Color(15, 15, 15));
                // leds.show();
                move.dont_move = true;
                break;
            }
            bot.dribblerAim();
            break;
        
        case State::Strategies::DEFEND:
            // Serial.println("defend");
            bot.triggerDefend();
            if(move.y > MAX_DEF_Y){
                move.y = MAX_DEF_Y;
            }
            break;

        case State::Strategies::BALLHIDE: //ballhide + decoy
            // Serial.println("ball hide");
            if(ball.ballCap && (millis() - ball.lastNoBallCap < ball.ballCapTime || drib.speed < drib.reach_speed)){
                leds.setPixelColor(7, leds.Color(15, 15, 15));
                leds.show();
                Serial.println("im not moving");
                move.dont_move = true;
                break;
            }
            // if(state.ready_to_shoot) {
            //     leds.setPixelColor(7, leds.Color(0, 0, 15));
            //     leds.show();
            //     bot.dribblerAim();
            //     break;
            // }
            bool goleft = false;
            if(espnowDataRecv.type == 0){ // other bot is off
                if(self.x < FIELD_WIDTH/2) goleft = true;
                else goleft = false;
            }
            else if(self.x < espnowDataRecv.xpos) goleft = true;
            else goleft = false;
            // if(self.x < espnowDataRecv.xpos){
            if(goleft){
                if (bot.ballHideLeft()){
                    state.ready_to_shoot = true; 
                    move.dont_move = true;
                    if(times.bhScore_timeout == 0){
                        times.bhScore_timeout = times.curTime;
                    } 
                }
                else{
                    state.ready_to_shoot = false;
                    times.bhScore_timeout = 0;
                }
            }
            // else if(self.x > espnowDataRecv.xpos){
            else{
                if (bot.ballHideRight()){
                    leds.setPixelColor(7, leds.Color(0, 15, 0));
                    leds.show();
                    state.ready_to_shoot = true; 
                    if(times.bhScore_timeout == 0){
                        times.bhScore_timeout = times.curTime;
                    }
                    move.dont_move = true;
                }
                else{
                    leds.setPixelColor(7, leds.Color(15, 0, 0));
                    leds.show();
                    state.ready_to_shoot = false;
                    times.bhScore_timeout = 0;
                }
            }
            break;
        case State::Strategies::BALLHIDE_LEAD:
            bot.ballHideLeader();
            break;
        case State::Strategies::BALLHIDE_FOLLOW:
            bot.ballHideFollower();
            break;
        case State::Strategies::ATTACK_MODE2: // single ballhide IMPT TESTT
            if(ball.ballCap && (millis() - ball.lastNoBallCap < ball.ballCapTime || drib.speed < drib.reach_speed)){
                move.dont_move = true;
                break;
            }
            if(state.topStrat == 1) bot.ballHideSide();
            else bot.ballHideMid();
            break;
        
        case State::Strategies::LOOK_AHEAD:
            bot.triggerLookAhead();
            break;
        
        case State::Strategies::ATTACK_BASIC:
            if(ball.ballCap && (millis() - ball.lastNoBallCap < ball.ballCapTime || drib.speed < drib.reach_speed)){
                move.dont_move = true;
                break;
            }
            else if(ball.ballCap) bot.newScoring();
            else if(ball.noBall) bot.moveToPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
            else bot.triggerLookAhead();
            break;
        
        case State::Strategies::DEFEND_BASIC:
            if(ball.ballCap){
                if(millis() - ball.lastNoBallCap > DEFENDER_WAIT_TIME) bot.newScoring();
                else move.dont_move = true;
            }
            else{
                if (ball.noBall || (ball.absolute_x > 0.62f && ball.absolute_x < 1.20f && ball.absolute_y < 0.25f))
                    bot.moveToPoint(FIELD_WIDTH/2, DEF_Y_DEFAULT, 0);
                else if (ball.absolute_y <  0.80f) bot.dribblerBallTrack();
                else bot.defend();
            }
            break;
        
        default:
            bot.moveToPoint(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
            break;
    }

    if (times.curTime - esp_last_send >= 250){
        sendData();
        esp_last_send = times.curTime;
    }

    // send moving command to motors
    if(switches.turnOff || switches.topOff) move.dont_move = true;
    if(move.dont_move) stop_motors();
    else movement(move.x, move.y, move.rotation);
    // DEBUG(move.x);
    // DEBUG(move.y);
    // DEBUG(move.rotation); // here

    // kicker check if can kick
    if(move.kick) {
        // check if can and want to kick, make dribbler backspin first
        bool can_kick = kicker.check_time();
        if(can_kick) drib.desired = -100;
    }

    // dribbler check current
    drib.analogval = dribblerMD.checkCurrent();
    drib.voltage = (drib.analogval / 4095) * 3.1;
    if(!drib.avg_filled){
        drib.v[drib.avg_cnt] = drib.voltage;
        drib.sum_avg += drib.v[drib.avg_cnt];
        drib.avgV = drib.sum_avg / (drib.avg_cnt+1);
    }
    else{
        drib.sum_avg -= drib.v[drib.avg_cnt];
        drib.v[drib.avg_cnt] = drib.voltage;
        drib.sum_avg += drib.v[drib.avg_cnt];
        drib.avgV = drib.sum_avg / drib.num_frames;
    }
    // Serial.printf("Average V: %f\n", drib.avgV);
    drib.avg_cnt++;
    if(!drib.avg_filled && drib.avg_cnt==drib.num_frames) drib.avg_filled = true;
    if(drib.avg_cnt>=drib.num_frames) drib.avg_cnt %= drib.num_frames;

    if(!drib.check_filled) {
        if(drib.avgV > drib.max_voltage) drib.c[drib.check_cnt] = 1;
        else drib.c[drib.check_cnt] = 0;
        drib.sum_check += drib.c[drib.check_cnt];
    }
    else{
        drib.sum_check -= drib.c[drib.check_cnt];
        if(drib.avgV > drib.max_voltage) drib.c[drib.check_cnt] = 1;
        else drib.c[drib.check_cnt] = 0;
        drib.sum_check += drib.c[drib.check_cnt];
    }
    drib.check_cnt++;
    if(!drib.check_filled && drib.check_cnt==drib.num_check) drib.check_filled = true;
    if(drib.check_cnt>=drib.num_check) drib.check_cnt %= drib.num_check;

    if(drib.sum_check >= drib.exceed_thresh * drib.num_check){
        if(drib.speed == 0) drib.speed = 0;
        else drib.speed -= copysign(drib.dec, drib.speed);
    }
    else{
        if(abs(drib.speed) < drib.minspeed) drib.speed = copysign(drib.minspeed, drib.desired);
        else{
            int diff = drib.desired - drib.speed;
            drib.speed += copysign(drib.inc, diff);
            if((drib.desired >= 0 && drib.speed > drib.desired) || (drib.desired < 0 && drib.speed < drib.desired))
                drib.speed = drib.desired;
        }
    }
    if(drib.desired == 0) drib.speed = 0;
    dribbler.setSpeed(drib.speed);
    // DEBUG(drib.desired);
    // DEBUG(drib.speed);

    // kicker - actually kick
    if(move.kick){
        bool kick_status = kicker.kick();
        if(kick_status){ // successfully kicked, reset state
            if(state.botID == 1){
                state.botType = 1;
            }
            else if(state.botID == 2){
                state.botType = 2;
            }
        }
    }

    // if(!(move.x == move.last_x && move.y == move.last_y && move.rotation == move.last_rotation)){
    //     state.strip++;
    // }
    // if(state.strip % 3 == 0) leds.setPixelColor(1, leds.Color(15, 0, 0));
    // else if (state.strip % 3 == 1) leds.setPixelColor(1, leds.Color(0, 15, 0));
    // else leds.setPixelColor(1, leds.Color(0, 0, 15));
    // state.strip %= 3;
    // leds.show();

    // update last type
    state.lastType = state.curType;

    ball.last_x = ball.absolute_x;
    ball.last_y = ball.absolute_y;
    ball.last_dist = ball.dist;
    
    move.last_x = move.x;
    move.last_y = move.y;
    move.last_rotation = move.rotation;
}