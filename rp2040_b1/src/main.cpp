#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Motor.h>
#include <MotorDriver.h>
#include <Drive.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define DATA_LEN 7
#define ADDR 0x09
byte buffer[DATA_LEN+1];

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define CS_PIN 1
#define NSLEEP_PIN 14
// #define NFAULT_PIN 7
#define DRVOFF_PIN 15
#define MOSI_PIN 3 // TX
#define MISO_PIN 0 // RX
#define SCK_PIN 2
MotorDriver motor_driver(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

uint8_t IN1_pin[NUM_DRIVERS] = {6, 8, 11, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {7, 9, 10, 12};
uint8_t IPROPI_pin[NUM_DRIVERS] = {26, 27, 28, 29}; // make sure pins can read analog

uint8_t maxspeed = 50;

Motor motorFL(IN1_pin[0], IN2_pin[0], maxspeed, 1.0);
Motor motorFR(IN1_pin[3], IN2_pin[3], maxspeed, 1.0);
Motor motorBL(IN1_pin[1], IN2_pin[1], maxspeed, 1.0);
Motor motorBR(IN1_pin[2], IN2_pin[2], maxspeed, 1.0);

Drive bot(motorFR, motorBR, motorBL, motorFL);
float speed = 1.0, moveAngle = 0.0, rotation = 0.0;

void receive(int num_bytes){
    if(num_bytes != DATA_LEN){
        Serial.print("Received bad data length");
        return;
    }
    else Serial.print("Received: ");
    for (int i=0; i<DATA_LEN; i++){
        if(Wire.available()){
            buffer[i] = Wire.read();
        }
    }
    buffer[DATA_LEN] = '\0';
    if(buffer[0]!=1 && buffer[0]!=0) {
        Serial.print("Received bad data");
        return;
    }
    if(buffer[0]==0){
        Serial.println("Bot off");
        bot.setDrive(0, 0, 0);
        return;
    }
    Serial.println("Received motor data");
    bool speedsign = true ? buffer[1]==1 : false;
    bool rotationsign = true ? buffer[3]==1 : false;
    speed = (float)(buffer[2]) / 255;
    if(!speedsign) speed *= -1;
    rotation = (float)(buffer[4]) / 255;
    if(!rotationsign) rotation *= -1;
    moveAngle = (float)(buffer[5] + (buffer[6]<<8)) / 128;
    bot.setDrive(1.0, moveAngle, rotation);
    // for (auto i : buffer){
    //     Serial.print(i);
    //     Serial.print(" ");
    // }
    // Serial.println();
}

void setup(){
    Serial.begin(115200);
    // while(!Serial.available()) ;
    // while(Serial.available()) Serial.read();
    // Serial.println("started");

    // pinMode(SDA_PIN, INPUT_PULLUP);
    // pinMode(SCL_PIN, INPUT_PULLUP);
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.setClock(100000);
    Wire.begin(ADDR);
    
    motor_driver.init();
    motor_driver.setMode();

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
}

void loop(){
    // Serial.println("running");
    strip.setPixelColor(0, strip.Color(0, 0, 15));
    strip.show();
    Wire.onReceive(receive);
    // Serial.print(moveAngle);
    // bot.setDrive(speed, moveAngle, rotation);
}

