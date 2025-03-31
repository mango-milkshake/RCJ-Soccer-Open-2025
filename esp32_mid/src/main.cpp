#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Kicker.h>

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
#define CAM_SERIAL_DATA_LEN 5
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
float relative_ball_x = 0, relative_ball_y = 0;
float absolute_ball_x = 0, absolute_ball_y = 0;
float cam_ball_x = 0, cam_ball_y = 0;
float line_status[NUM_LINE_MUX];
bool noBall = false, ballCap = false, isTilted = false, isOnLine = false;
float lastLoopTime = 0, lastBallCap = 0;
float speed_xdir, speed_ydir, rotation;

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
            ball_angle = (float)(uartBufferCam[1] + (uartBufferCam[2]<<8)) / 128;
            ball_dist = (float)(uartBufferCam[3] + (uartBufferCam[4]<<8)) / 128;
            if(ball_angle==0 && ball_dist==0) {
                noBall = true;
                setLED(6, 8, strip.Color(0, 0, 15));
            }
            else {
                noBall = false;
                setLED(6, 8, strip.Color(0, 15, 0));
            }

            if(noBall) dribbler.setSpeed(0);
            else if(ball_dist<=40) dribbler.setSpeed(1.0);
            else dribbler.setSpeed(0.5);

            float relative_angle = 90 - (ball_angle + self_heading);
            // ball_dist += BOT_RADIUS;
            relative_ball_x = (ball_dist * cosf(RAD(relative_angle))) / 100;
            relative_ball_y = (ball_dist * sinf(RAD(relative_angle))) / 100;
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

    cam_ball_x = (float)(rcvBuffer[2] + (rcvBuffer[3]<<8)) / 128;
    if(rcvBuffer[1]==0) cam_ball_x *= -1;
    cam_ball_y = (float)(rcvBuffer[4] + (rcvBuffer[5]<<8)) / 128;

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

    if(noBall) movement(FIELD_WIDTH/2, FIELD_HEIGHT/2, 0);
    else if(ballCap) aim();
    else ballTrack();
}
