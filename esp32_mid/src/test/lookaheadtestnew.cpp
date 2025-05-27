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
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
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

// I2C Comms with middle plate RP2040
#define MID_SDA_PIN 1
#define MID_SCL_PIN 2
#define MID_I2C_SEND_DATA_LEN 8
#define MID_I2C_RCV_DATA_LEN 5
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
#define LAST_SEEN_BALL_TIME 1000
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

//// ** FUNCTIONS ** ////

void sendMidPlateData(){
    Wire.beginTransmission(MID_I2C_ADDR);
    Wire.write(midSendBuffer, MID_I2C_SEND_DATA_LEN);
    Wire.endTransmission();
}


void getMidPlateData(){
    byte num_bytes = Wire.requestFrom(MID_I2C_ADDR, MID_I2C_RCV_DATA_LEN);
    if(num_bytes != MID_I2C_RCV_DATA_LEN){
        Serial.print("Received bad data: ");
        return;
    }
    else Serial.print("Received: ");
    for (int i=0; i<MID_I2C_RCV_DATA_LEN; i++) {
        if (Wire.available()) {
            midRcvBuffer[i] = Wire.read();
        }
    }
    if(midRcvBuffer[0]!=firstbyte) {
        Serial.print("Received bad data");
        return;
    }
    ball_angle = (float)(midRcvBuffer[1] + (midRcvBuffer[2]<<8)) / 128;
    ball_dist = (float)(midRcvBuffer[3] + (midRcvBuffer[4]<<8)) / 128;

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
void updateSelfVelocityEWMA(float current_self_w, float current_self_x, float current_self_y) { 
    unsigned long now = millis();
    float dt = (now - last_vel_time) / 1000.0f; 
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
    float inst_ball_vx = (absolute_ball_x - last_top_absolute_ball_x) / dt;
    float inst_ball_vy = (absolute_ball_y - last_top_absolute_ball_y) / dt;
    DEBUG(inst_ball_vx);
    DEBUG(inst_ball_vy);
    DEBUG(absolute_ball_x);
    DEBUG(absolute_ball_y);

    // Exponential Weighted Moving Average update, beta parameter used = 0.8
    self_velocityw = 0.2f * inst_vw + (0.8f) * self_velocityw;    
    self_velocityx = 0.2f * inst_vx + (0.8f) * self_velocityx;
    self_velocityy = 0.2f * inst_vy + (0.8f) * self_velocityy;
    if (!noBall && abs(pow((inst_ball_vx*inst_ball_vx+inst_ball_vy*inst_ball_vy),0.5)) < 2){    
        ball_vx = 0.2f * inst_ball_vx + (0.8f) * ball_vx;
        ball_vy = 0.2f * inst_ball_vy + (0.8f) * ball_vy;
    }
    else{
        Serial.println("data cancelled");
    }

    // Save current data for next iteration
    last_self_w = current_self_w;
    last_self_x = current_self_x;
    last_self_y = current_self_y;
    last_top_absolute_ball_x = absolute_ball_x;
    last_top_absolute_ball_y = absolute_ball_y;
    last_vel_time = now;
    DEBUG(self_velocityx);
    DEBUG(self_velocityy);
    DEBUG(ball_vx);
    DEBUG(ball_vy);
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
        DEBUG(turnOff);
        DEBUG(topOff);
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


float targetballposx = FIELD_WIDTH/2;
float targetballposy = 0.80;
float targetheadinglookahead = 0;
void lookAhead(){
    float v = 3;
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
        LAball_vx *= 0.95;
        LAball_vy *= 0.95;
        //sendI2C(zeroBuffer);
        //dribblerBallTrack();
        lookAheadConfirm = false;        
    } 
} 
    
    targetballposx = LAball_x + LAball_vx*t;
    targetballposy = LAball_y + LAball_vy*t;
    targetheadinglookahead = atan2(targetballposy,targetballposx);
    targetballposx += self_x; 
    targetballposy += self_y;

    DEBUG(targetballposx);
    DEBUG(targetballposy);
    // DEBUG(LAball_vx);
    // DEBUG(LAball_vy);
    // DEBUG(self_x);
    // DEBUG(self_y);
    DEBUG(t);
    // DEBUG(lookAheadConfirm);

    if (lookAheadConfirm){
        // movement(targetballposx, targetballposy, 0);
        Serial.println("Moving to new target");
    }    
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

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

}


float lastLookAhead = millis();
float targetballposx_current = 0;
float targetballposy_current = 0;
#define LOOK_AHEAD_THRESHOLD_T 0
#define LOOK_AHEAD_THRESHOLD_DMIN 0
#define LOOK_AHEAD_THRESHOLD_DMAX 2
bool rotateBot_LA = true;
void loop(){
    // Serial.println("running main code");
    float curTime = millis();
    //Serial.print("time: ");
    //Serial.println(curTime - lastLoopTime);
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
    /*
    DEBUG(pbvx[pbvx_size]);
    DEBUG(pbvx[pbvx_size-1]);
    DEBUG(pbvx[pbvx_size-2]);
    DEBUG(pbvy[pbvx_size]);
    DEBUG(pbvy[pbvx_size-1]);
    DEBUG(pbvy[pbvx_size-2]);
    DEBUG(noBall);*/
    updateSelfVelocityEWMA(RAD(self_heading), self_x, self_y); 
    if(ballCap){
        turnOff = true;
        noBallTimer = 0;
    }
    else{
        turnOff = false;
        if(noBall){
            if(noBallTimer == 0){ //substitute for remembering last ball location in main code
                noBallTimer = millis(); 
            }
            else if(abs(millis() - noBallTimer) < 2000){
                absolute_ball_x = last_ball_x;
                absolute_ball_y = last_ball_y;
            }
            else{
                moveToGoal = true;
            }
        }
        else{
            last_ball_x = absolute_ball_x;
            last_ball_y = absolute_ball_y;
            noBallTimer = 0;
            moveToGoal = false;
        }
        
        if(!moveToGoal && noBall && checkzero(10)){ // if no ball, stop bot
            // esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
            // sendI2C(zeroBuffer);
            // esp_led.show();
            // lookAhead(); //delete this later if needed
        }  
        // else if (sqrtf(ball_vx*ball_vx + ball_vy*ball_vy) < 0.15){} //don't look ahead if velocity is too small
        else if (!moveToGoal && !noBall && checkv(4, 200)){ // if last n values are within x of each other, update look ahead target
            if(abs(curTime - lastLookAhead) > LOOK_AHEAD_THRESHOLD_T){
                //switches target only if last switch target was sufficiently long ago
                esp_led.setPixelColor(0, esp_led.Color(0, 20, 0));
                esp_led.show();
                Serial.println("look ahead called////////////////////////////////////////////////////////////");
                lookAhead();
                lastLookAhead = millis();
            }
        } 

        if (moveToGoal){
            if(targetballposx != FIELD_WIDTH/2 && targetballposy != 0.5){
                // sendI2C(zeroBuffer)
            }
            targetballposx = FIELD_WIDTH/2;
            targetballposy = 0.5;

            movement(targetballposx, targetballposy, 0);
        }
        // DEBUG(moveToGoal);
        DEBUG(targetballposx);
        DEBUG(targetballposy);
        // DEBUG(targetballposx_current);
        // DEBUG(targetballposy_current);
        // DEBUG(self_x);
        // DEBUG(self_y);
        float ballAngle_LA = rotateBot_LA ? 90-DEG(atan2(relative_ball_y, relative_ball_x)) : 0;
        float LA_distchange = pow((targetballposx*targetballposx + targetballposy*targetballposy),0.5) - pow((targetballposx_current*targetballposx_current + targetballposy_current*targetballposy_current),0.5);
        //call movement at least once every loop
        if(!moveToGoal && (targetballposx<0 || targetballposx>FIELD_WIDTH || targetballposy<0 || targetballposy>FIELD_HEIGHT)){
            targetballposx = targetballposx_current;
            targetballposy = targetballposy_current;
            // targetheadinglookahead = 0;
            movement(targetballposx, targetballposy, ballAngle_LA);
            // sendI2C(zeroBuffer);
        }
        else if (!moveToGoal && abs(LA_distchange) >= LOOK_AHEAD_THRESHOLD_DMIN && abs(LA_distchange) <= LOOK_AHEAD_THRESHOLD_DMAX){
            //switches target only if new target is far away from current target 
            targetballposx_current = targetballposx;
            targetballposy_current = targetballposy; 
            // targetheadinglookahead = 0;
            movement(targetballposx, targetballposy, ballAngle_LA);
        }
    }   
    /* //code from main.cpp
    // int rounded_coord_x = floor(self_x * 128);
    // int rounded_coord_y = floor(self_y * 128);
    // int uart_heading = floor(abs(self_heading) * 128);

    // midSendBuffer[0] = 5;
    // midSendBuffer[1] = rounded_coord_x & 0xFF;
    // midSendBuffer[2] = (rounded_coord_x >> 8) & 0xFF;
    // midSendBuffer[3] = rounded_coord_y & 0xFF;
    // midSendBuffer[4] = (rounded_coord_y >> 8) & 0xFF;

    // if(copysign(1, self_heading)==1) midSendBuffer[5] = 1;
    // else midSendBuffer[5] = 0;
    // midSendBuffer[6] = uart_heading & 0xFF;
    // midSendBuffer[7] = (uart_heading >> 8) & 0xFF;
    // sendMidPlateData();

    // if(noBall) movement(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
    // else movement(absolute_ball_x, absolute_ball_y, 0); */
}
