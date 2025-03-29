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

// ESP NeoPixel LED
#define ESP_LED 21
#define ESP_BRIGHTNESS 50
#define BLINK_TIME 1000
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);
bool esp_led_state = false;
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
#define FIELD_WIDTH 1.82 // 0.91
#define FIELD_HEIGHT 2.43 // 1.21
#define BALL_CAP_THRESH 15 // in cm

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

// PID
PID pid_rotate(0.35, 0, 0, 1000);
PID pid_x(4.5, 0, 0, 5000);
PID pid_y(4.5, 0, 0, 5000);

// Mutexes
SemaphoreHandle_t i2cMutex, coordMutex;

// Variables - access from both cores
float cur_x, cur_y, cur_heading;
bool curTilt = false;

// Variables - core 0 only
float self_x = 0, self_y = 0, self_heading = 0;
float speed_xdir, speed_ydir, rotation;
bool selfTilt = false;
float lastTime = 0;
float targetx[8] = {0.91, 0.30, 0.91, 1.52, 0.91, 0.30, 0.91, 1.52};
float targety[8] = {1.21, 2.13, 1.21, 2.13, 1.21, 0.30, 1.21, 0.30};
int targetidx = 0;
bool reached = false;
float reachTime = 0;

// Variables - core 1 only
float coord_x = 0, coord_y = 0, heading = 0;
bool isTilted = false;

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
            coord_x = (float)(uartBufferPico[1] + (uartBufferPico[2]<<8)) / 128;
            coord_y = (float)(uartBufferPico[3] + (uartBufferPico[4]<<8)) / 128;
            heading = (float)(uartBufferPico[6] + (uartBufferPico[7]<<8)) / 128;
            if(uartBufferPico[5]==0) heading *= -1;
            if(uartBufferPico[8]==1) isTilted = true;
            else isTilted = false;

            if(xSemaphoreTake(coordMutex, portMAX_DELAY)){ // here
                cur_x = coord_x;
                cur_y = coord_y;
                cur_heading = heading;
                curTilt = isTilted;
                xSemaphoreGive(coordMutex);
                // Serial.println("Coordinate data updated");
            }

            // DEBUG(heading);
            setLED(1, 2, strip.Color(15, 0, 15));
        }
    }
    else setLED(1, 2, strip.Color(0, 15, 15));
}

void updateData(){
    if(xSemaphoreTake(coordMutex, 0)){
        self_x = cur_x;
        self_y = cur_y;
        self_heading = cur_heading;
        selfTilt = curTilt;
        xSemaphoreGive(coordMutex);
        // Serial.println("Updated coordinates");
        // DEBUG(self_x);
        // DEBUG(self_y);
    }
}

void sendI2C(byte (&buffer)[I2C_SEND_DATA_LEN]){
    if(xSemaphoreTake(i2cMutex, portMAX_DELAY)){ // here
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
    float total_dist = sqrt(x_dist*x_dist + y_dist*y_dist);
    float total_angle = atan2(y_dist, x_dist) + RAD(self_heading) - PI/4; // in radians
    if(total_dist <= 0.03) {
        if(!reached) {
            reached = true;
            reachTime = millis();
        }
    }
    if(reached && millis() - reachTime >= 100){
        targetidx++;
        targetidx %= 8;
        reached = false;
        reachTime = 0;
    }
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

    if(turnOff || selfTilt) sendI2C(zeroBuffer);
    else sendI2C(sendBuffer);
}

//// ** LOOPS ** ////

// core 0 handles main game logic and writing motor control info to rp2040
void core0Task(void *pvParameters){
    zeroBuffer[0] = 0;
    for (int i=1; i<I2C_SEND_DATA_LEN; i++) zeroBuffer[i] = 0;
    while(1){
        // Serial.print("Core0");
        float curTime = millis();
        Serial.println("time: ");
        Serial.println(curTime - lastTime);
        lastTime = millis();
        if(digitalRead(TURN_OFF_SW)==HIGH) turnOff = true;
        else turnOff = false;

        updateData();

        float angleToFace = 90 - DEG(atan2(1.215 - self_y, 0.91 - self_x));

        movement(targetx[targetidx], targety[targetidx], angleToFace);
    }
}

// core 1 handles receiving data and processing to get final self and ball coordinates
void core1Task(void *pvParameters){
    while(1){
        // Serial.print("Core1");
        if(millis() - lastLED >= BLINK_TIME){
            esp_led_state = !esp_led_state;
            lastLED = millis();
        }
        if(esp_led_state){
            esp_led.setPixelColor(0, esp_led.Color(50, 50, 0));
            esp_led.show();
        }
        else{
            esp_led.setPixelColor(0, esp_led.Color(0, 0, 0));
            esp_led.show();
        }

        // readVoltage();
        getTopPlateData();
    }
}

void setup(){
    Serial.begin(115200);

    Serial2.begin(115200, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);

    pinMode(TURN_OFF_SW, INPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    analogSetAttenuation(ADC_11db);

    i2cMutex = xSemaphoreCreateMutex(); 
    coordMutex = xSemaphoreCreateMutex();
    Wire.begin(SDA_PIN, SCL_PIN, 50000);

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show();

    esp_led.begin();
    esp_led.setBrightness(ESP_BRIGHTNESS);
    esp_led.show();

    xTaskCreatePinnedToCore(core0Task, "Send Data", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Read Data", 16384, NULL, 1, NULL, 1);
}

void loop(){
    
}
