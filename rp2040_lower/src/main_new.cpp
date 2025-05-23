#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Motor.h>
#include <MotorDriver.h>
#include <Drive.h>
#include <UARTComms.h>

#define TX_PIN 4
#define RX_PIN 5
#define DATA_LEN 7
byte buffer[DATA_LEN];
UARTComms espUART(TX_PIN, RX_PIN, buffer, DATA_LEN, Serial2);

#define led_pin 16
#define led_count 1
#define brightness 50
#define BLINK_TIME 1000
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);
bool led_state = true;
float lastLED = 0;

#define CS_PIN 1
#define NSLEEP_PIN 14
#define DRVOFF_PIN 15
#define MOSI_PIN 3 // TX
#define MISO_PIN 0 // RX
#define SCK_PIN 2
MotorDriver motor_driver(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

// #define SECOND_BOT // = bot 1
#ifdef SECOND_BOT 
uint8_t IN1_pin[NUM_DRIVERS] = {6, 9, 11, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {7, 8, 10, 12};
uint8_t NFAULT_pin[NUM_DRIVERS] = {26, 27, 28, 29};
#else
uint8_t IN1_pin[NUM_DRIVERS] = {7, 9, 11, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {6, 8, 10, 12};
uint8_t NFAULT_pin[NUM_DRIVERS] = {26, 27, 28, 29};
#endif

uint8_t maxspeed = 80;

Motor motorFL(IN1_pin[0], IN2_pin[0], NFAULT_pin[0], maxspeed, 1.0);
Motor motorFR(IN1_pin[3], IN2_pin[3], NFAULT_pin[3], maxspeed, 1.0);
Motor motorBL(IN1_pin[1], IN2_pin[1], NFAULT_pin[1], maxspeed, 1.0);
Motor motorBR(IN1_pin[2], IN2_pin[2], NFAULT_pin[2], maxspeed, 1.0);
float lastFault = 0;

Drive bot(motorFR, motorBR, motorBL, motorFL);
float speedX = 0.0, speedY = 0.0, speed_xdir = 0.0, speed_ydir = 0.0, moveAngle = 0.0, rotation = 0.0;

void checkFault(){
    bool faulted = false;
    if(digitalRead(motorFL.nfault)==LOW) faulted = true;
    if(digitalRead(motorFR.nfault)==LOW) faulted = true;
    if(digitalRead(motorBL.nfault)==LOW) faulted = true;
    if(digitalRead(motorBR.nfault)==LOW) faulted = true;
    if(faulted){
        // Serial.println("faulted");
        float curTime = millis();
        if(curTime - lastFault >= 500){
            motor_driver.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
}

void setup(){
    Serial.begin(115200);

    espUART.init();
    
    motor_driver.init();
    motor_driver.setMode();

    strip.begin();
    strip.setBrightness(brightness);
    strip.setPixelColor(0, strip.Color(15, 15, 0));
    strip.show();
}

void loop(){
    if(millis() - lastLED >= BLINK_TIME){
        led_state = !led_state;
        lastLED = millis();
    }
    if(led_state){
        strip.setPixelColor(0, strip.Color(0, 0, 15));
        strip.show();
    }
    else{
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
    }
    checkFault();
    bool status = espUART.uartRead((byte)5);
    if(status){
        bool speed_x_sign = buffer[1]==1 ? true : false;
        bool speed_y_sign = buffer[3]==1 ? true : false;
        bool rotationsign = buffer[5]==1 ? true : false;
        speed_xdir = (float)(buffer[2]) / 255;
        if(!speed_x_sign) speed_xdir *= -1;
        speed_ydir = (float)(buffer[4]) / 255;
        if(!speed_y_sign) speed_ydir *= -1;
        rotation = (float)(buffer[6]) / 255;
        if(!rotationsign) rotation *= -1;

        speedX = speed_xdir * cosf(RAD(135)) + speed_ydir * cosf(RAD(45));
        speedY = speed_xdir * sinf(RAD(135)) + speed_ydir * sinf(RAD(45));
    }
    bot.setDrive(speedX, speedY, rotation);
    // bot.setDrive(speed_xdir, speed_ydir, rotation);
}
