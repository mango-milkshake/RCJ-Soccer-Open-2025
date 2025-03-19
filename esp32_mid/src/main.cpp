#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <CommonUtils.h>
#include <Adafruit_NeoPixel.h>

#define DEBUGGING
#ifdef DEBUGGING
#define DEBUG(x) Serial.println(String(#x) + String(": ") + String(x) + String('\r')); 
#else
#define DEBUG(x) 123;
#endif

//// ** DEFINITIONS ** ////

// Debug LEDs
#define LED_PIN 18
#define LED_COUNT 10
#define LED_BRIGHTNESS 100
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor software switch
#define TURN_OFF_SW 40
bool turnOff = false;

// Dimensions
#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21
#define BALL_CAP_THRESH 12 // in cm

// I2C Comms with bottom plate
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_RCV_DATA_LEN 32
#define I2C_SEND_DATA_LEN 9
#define I2C_RCV_PICO_ADDR 0x08
#define I2C_SEND_PICO_ADDR 0x09
byte rcvBuffer[I2C_RCV_DATA_LEN+1], sendBuffer[I2C_SEND_DATA_LEN];
byte zeroBuffer[I2C_SEND_DATA_LEN];

// UART Comms with top plate
#define PICO_TX_PIN 16
#define PICO_RX_PIN 17
#define PICO_SERIAL_DATA_LEN 10
byte uartBufferPico[PICO_SERIAL_DATA_LEN];

// UART Comms with camera
#define CAM_TX_PIN 10
#define CAM_RX_PIN 11
#define CAM_SERIAL_DATA_LEN 5
byte uartBufferCam[CAM_SERIAL_DATA_LEN];

// PID
PID pid_rotate(2, 0, 0, 1000);
PID pid_x(6, 0, 0, 5000);
PID pid_y(6, 0, 0, 5000);

// Mutexes
SemaphoreHandle_t i2cMutex, coordMutex, ballMutex;

// Variables - access from both cores
float cur_x, cur_y, cur_lidar_heading, cur_imu_heading;
float cur_ball_x = 0, cur_ball_y = 0;
bool curBallCap = false;

// Variables - core 0 only
float self_x = 0, self_y = 0, self_lidar_heading = 0, self_imu_heading = 0;
float self_ball_x = 0, self_ball_y = 0, self_ball_angle = 0;
float speed_xdir, speed_ydir, rotation;
bool selfBallCap = false;

// Variables - core 1 only
float coord_x = 0, coord_y = 0, lidar_heading = 0, imu_heading = 0;
float ball_angle = 0, ball_dist = 0, ball_x = 0, ball_y = 0;
bool no_ball = false, ballCap = false;
float lastBallCap = 0;

//// ** FUNCTIONS ** ////

void setLED(int first, int last, uint32_t color){
    for (int i=first; i<=last; i++){
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void getTopPlateData(){
    if(Serial2.available()>=PICO_SERIAL_DATA_LEN){
        while(Serial2.peek()!=1) {
            Serial.println("Pico first byte not 1");
            Serial2.read();
        }
        int len = Serial2.readBytes(uartBufferPico, PICO_SERIAL_DATA_LEN);
        if(len!=PICO_SERIAL_DATA_LEN || uartBufferPico[0]!=1){
            Serial.print("Received bad data: length: ");
            Serial.print(len);
            Serial.print(", data: ");
            for (auto i : uartBufferPico) {
                Serial.print(i);
                Serial.print(" ");
            }
            setLED(0, LED_COUNT/2-1, strip.Color(15, 0, 15));
        }
        else{
            coord_x = (float)(uartBufferPico[1] + (uartBufferPico[2]<<8)) / 128;
            coord_y = (float)(uartBufferPico[3] + (uartBufferPico[4]<<8)) / 128;
            lidar_heading = (float)(uartBufferPico[5] + (uartBufferPico[6]<<8)) / 128;

            imu_heading = (float)(uartBufferPico[8] + (uartBufferPico[9]<<8)) / 128;
            if(uartBufferPico[7]==0) imu_heading *= -1;

            if(xSemaphoreTake(coordMutex, portMAX_DELAY)){
                cur_x = coord_x;
                cur_y = coord_y;
                cur_lidar_heading = lidar_heading;
                cur_imu_heading = imu_heading;
                xSemaphoreGive(coordMutex);
                // Serial.println("Coordinate data updated");
            }
            setLED(0, LED_COUNT/2-1, strip.Color(0, 15, 15));
        }
    }
}

void getTopCamData(){
    if(Serial1.available()>=CAM_SERIAL_DATA_LEN){
        while(Serial1.peek()!=1) {
            Serial.println("Camera first byte not 1");
            Serial1.read();
        }
        int len = Serial1.readBytes(uartBufferCam, CAM_SERIAL_DATA_LEN);
        Serial.println(len);
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
            Serial.print(ball_angle);
            ball_dist = (float)(uartBufferCam[3] + (uartBufferCam[4]<<8)) / 128;
            if(!no_ball && (ball_angle <= 15 || ball_angle >= 345) && ball_dist <= BALL_CAP_THRESH){
                ballCap = true;
                lastBallCap = millis();
            }
            else {
                ballCap = false;
            }

            float relative_angle = 90 - (ball_angle + imu_heading);
            ball_x = (ball_dist * cosf(RAD(relative_angle))) / 100;
            ball_y = (ball_dist * sinf(RAD(relative_angle))) / 100;

            if(no_ball) setLED(LED_COUNT/2, LED_COUNT-1, strip.Color(0, 0, 15));
            else setLED(LED_COUNT/2, LED_COUNT-1, strip.Color(0, 15, 0));

            if(xSemaphoreTake(ballMutex, portMAX_DELAY)){
                if(no_ball){
                    cur_ball_x = 0;
                    cur_ball_y = 0;
                }
                else{
                    cur_ball_x = ball_x;
                    cur_ball_y = ball_y;
                }
                curBallCap = ballCap;
                xSemaphoreGive(ballMutex);
                // DEBUG(ball_x);
                // DEBUG(ball_y);
                // Serial.println("Ball data updated");
            }
        }
    }
}

void updateData(){
    if(xSemaphoreTake(coordMutex, 0)){
        self_x = cur_x;
        self_y = cur_y;
        self_lidar_heading = cur_lidar_heading;
        self_imu_heading = cur_imu_heading;
        xSemaphoreGive(coordMutex);
        // Serial.println("Updated coordinates");
        // DEBUG(self_x);
        // DEBUG(self_y);
    }

    if(xSemaphoreTake(ballMutex, 0)){
        self_ball_x = cur_ball_x;
        self_ball_y = cur_ball_y;
        selfBallCap = curBallCap;
        xSemaphoreGive(ballMutex);
    }
}

void sendI2C(byte (&buffer)[I2C_SEND_DATA_LEN]){
    if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
        Wire.beginTransmission(I2C_SEND_PICO_ADDR);
        Wire.write(buffer, I2C_SEND_DATA_LEN);
        Wire.endTransmission();
        xSemaphoreGive(i2cMutex);
        // Serial.println("Data sent");
    }
}

void movement(float target_x, float target_y, float target_rotation){
    target_x = constrain(target_x, 0.12, FIELD_WIDTH - 0.12);
    target_y = constrain(target_y, 0.37, FIELD_HEIGHT - 0.37);
    float x_dist = target_x - self_x, y_dist = target_y - self_y;
    float rotation_dist = self_imu_heading - target_rotation;
    while(rotation_dist > 180) rotation_dist -= 360;
    while(rotation_dist < -180) rotation_dist += 360;
    speed_xdir = constrain(pid_x.compute(0, x_dist), -1, 1);
    speed_ydir = constrain(pid_y.compute(0, y_dist), -1, 1);
    rotation = constrain(pid_rotate.compute(0, RAD(rotation_dist)), -1, 1);

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

    if(turnOff) sendI2C(zeroBuffer);
    else sendI2C(sendBuffer);
}

//// ** LOOPS ** ////

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
    zeroBuffer[0] = 0;
    for (int i=1; i<I2C_SEND_DATA_LEN; i++) zeroBuffer[i] = 0;
    while(1){
        // Serial.print("Core0");
        if(digitalRead(TURN_OFF_SW)==HIGH) turnOff = true;
        else turnOff = false;

        updateData();

        float x_dist = target_x - self_x, y_dist = target_y - self_y;
        speed_xdir = constrain(pid_x.compute(0, x_dist), -1, 1);
        speed_ydir = constrain(pid_y.compute(0, y_dist), -1, 1);
        // float distance = sqrt(x_dist * x_dist + y_dist * y_dist);

        if(x_dist==0) {
            if(y_dist>=0) angle = 90;
            else angle = 270;
         }
         else if(y_dist==0){
             if(x_dist>=0) angle = 0;
             else angle = 180;
         }
         else{
             angle = DEG(atanf(abs(y_dist / x_dist)));
             if(y_dist>0 && x_dist<0) angle = 180 - angle;
             else if(y_dist<0 && x_dist<0) angle = 180 + angle;
             else if(y_dist<0 && x_dist>0) angle = 360 - angle;
         }

        angle = 90 - angle;
        if(angle<0) angle += 360;
        if(angle>=360) angle -= 360;
        rotation = constrain(pid_rotate.compute(0, RAD(self_imu_heading)), -1, 1);
        // speed = constrain(pid_speed.compute(0, distance), -1, 1);
        // DEBUG(rotation);
        // DEBUG(speed); 
        // DEBUG(angle);

        angle = angle - self_imu_heading;
        if(angle<0) angle += 360;
        if(angle>=360) angle -= 360;

        // DEBUG(speed_xdir);
        // DEBUG(speed_ydir);

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
        int rounded_angle = floor(angle * 128);

        sendBuffer[0] = 5;
        sendBuffer[1] = speed_x_sign;
        sendBuffer[2] = rounded_speed_x;
        sendBuffer[3] = speed_y_sign;
        sendBuffer[4] = rounded_speed_y;
        sendBuffer[5] = rotation_sign;
        sendBuffer[6] = rounded_rotation;
        sendBuffer[7] = (rounded_angle & 0xFF);
        sendBuffer[8] = ((rounded_angle >> 8) & 0xFF);

        if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){
            Wire.beginTransmission(I2C_SEND_PICO_ADDR);
            if(turnOff) {
                // Serial.println("Bot off");
                Wire.write(zeroBuffer, I2C_SEND_DATA_LEN);
            }
            else Wire.write(sendBuffer, I2C_SEND_DATA_LEN);
            Wire.endTransmission();
            xSemaphoreGive(i2cMutex);
            // Serial.println("Data sent");
        }
    }
}

// core 1 handles receiving data and processing to get final self and ball coordinates
void core1Task(void *pvParameters){
    while(1){
        // Serial.print("Core1");
        getTopPlateData();
        getTopCamData();
    }
}

void setup(){
    Serial.begin(115200);

    Serial1.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    Serial2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);

    i2cMutex = xSemaphoreCreateMutex(); 
    coordMutex = xSemaphoreCreateMutex();
    ballMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN, 100000);

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    xTaskCreatePinnedToCore(core0Task, "Send Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Read Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    
}
