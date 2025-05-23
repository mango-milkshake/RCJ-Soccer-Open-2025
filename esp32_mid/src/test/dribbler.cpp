#include <Arduino.h>
#include <DribblerNew.h>
#include <Motor.h>
#include <Adafruit_NeoPixel.h>

// Dribbler
#define MOSI_PIN 12
#define MISO_PIN 13
#define SCK_PIN 14
#define CS_PIN 15
#define DRIBBLER_IN1 48
#define DRIBBLER_IN2 47
#define DRIBBLER_NFAULT 21
#define NSLEEP_PIN 6
#define DRVOFF_PIN 7
#define IPROPI_PIN 5
MotorDriver dribblerMD(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN, IPROPI_PIN);

uint8_t dribbler_maxspeed = 80;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
float lastFault = 0;

// LEDs
#define ESP_LED 48
#define ESP_BRIGHTNESS 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);

// Voltage averaging
#define NUM_FRAMES 60
int counter = 0;
float v[NUM_FRAMES];
bool filled = false;
float sum = 0, avgV = 0;

#define MAX_VOLTAGE 0.5
#define STOP_TIME 2000

void checkFault(){
    bool faulted = false;
    if(digitalRead(dribbler.nfault)==LOW) faulted = true;
    if(faulted){
        Serial.println("faulted");
        float curTime = millis();
        if(curTime - lastFault >= 500){
            dribblerMD.clearFault();
            dribblerMD.readRegister(0b01000001);
            lastFault = millis();
        }
    } 
}

void setup(){
    Serial.begin(115200);
    // delay(1000);

    // while(!Serial.available());
    // while(Serial.available()) Serial.read();
    Serial.println("started");

    // esp_led.begin();
    // esp_led.setBrightness(ESP_BRIGHTNESS);
    // esp_led.show();

    dribblerMD.init();
    dribblerMD.setMode();

    // while(!Serial.available());
    // while(Serial.available()) Serial.read();
    for (int i=0; i<NUM_FRAMES; i++) v[i] = 0;
}

void loop(){
    // Serial.println("running");
    // esp_led.setPixelColor(0, esp_led.Color(0, 15, 0));
    // esp_led.show();
    checkFault();
    dribbler.setSpeed(-1.0);
    float analogval = dribblerMD.checkCurrent();
    // Serial.printf("Analog value: %f\n", analogval);
    float voltage = (analogval / 4095) * 3.1;
    // Serial.printf("Voltage: %f\n", voltage);
    if(!filled){
        v[counter] = voltage;
        sum += v[counter];
        avgV = sum / (counter+1);
    }
    else{
        sum -= v[counter];
        v[counter] = voltage;
        sum += v[counter];
        avgV = sum / NUM_FRAMES;
    }
    Serial.printf("Average V: %f\n", avgV);
    counter++;
    if(!filled && counter==NUM_FRAMES) filled = true;
    if(counter>=NUM_FRAMES) counter %= NUM_FRAMES;
    if(avgV > MAX_VOLTAGE){
        dribbler.setSpeed(0);
        delay(STOP_TIME);
        // reset everything to default
        counter = 0;
        filled = false;
        sum = 0;
        avgV = 0;
        for (int i=0; i<NUM_FRAMES; i++) v[i] = 0;
    }
}