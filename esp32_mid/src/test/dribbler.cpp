#include <Arduino.h>
#include <Dribbler.h>
#include <Motor.h>
#include <Adafruit_NeoPixel.h>

#define TESTING

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

uint8_t dribbler_maxspeed = 150, dribbler_minspeed = 40;
Motor dribbler(DRIBBLER_IN1, DRIBBLER_IN2, DRIBBLER_NFAULT, dribbler_maxspeed, 1.0);
int lastFault = 0;
int lastStop = 0;
bool stopped = false;
int desiredSpeed = dribbler_maxspeed, dribblerSpeed = dribbler_maxspeed;

// LEDs
#define ESP_LED 48
#define ESP_BRIGHTNESS 50
Adafruit_NeoPixel esp_led(1, ESP_LED, NEO_GRB + NEO_KHZ800);

// Voltage averaging
#define NUM_FRAMES 5
#define NUM_CHECK 10
#define EXCEED_THRESH 0.70
int avg_cnt = 0, check_cnt = 0;
float v[NUM_FRAMES];
int c[NUM_CHECK];
bool avg_filled = false, check_filled = false;
float sum_avg = 0, sum_check = 0, avgV = 0;

#define MAX_VOLTAGE 0.1
#define INC_AMT 1
#define DEC_AMT 2

void checkFault(){
    bool faulted = false;
    if(digitalRead(dribbler.nfault)==LOW) faulted = true;
    if(faulted){
        Serial.println("faulted");
        int curTime = millis();
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

    #ifdef TESTING
    dribbler.setSpeed(dribbler_maxspeed);
    delay(1);
    return;
    #endif

    dribbler.setSpeed(dribblerSpeed);

    float analogval = dribblerMD.checkCurrent();
    // Serial.printf("Analog value: %f\n", analogval);
    float voltage = (analogval / 4095) * 3.1;
    // Serial.printf("Voltage: %f\n", voltage);
    if(!avg_filled){
        v[avg_cnt] = voltage;
        sum_avg += v[avg_cnt];
        avgV = sum_avg / (avg_cnt+1);
    }
    else{
        sum_avg -= v[avg_cnt];
        v[avg_cnt] = voltage;
        sum_avg += v[avg_cnt];
        avgV = sum_avg / NUM_FRAMES;
    }
    Serial.printf("Average V: %f\n", avgV);
    avg_cnt++;
    if(!avg_filled && avg_cnt==NUM_FRAMES) avg_filled = true;
    if(avg_cnt>=NUM_FRAMES) avg_cnt %= NUM_FRAMES;

    if(!check_filled) {
        if(avgV > MAX_VOLTAGE) c[check_cnt] = 1;
        else c[check_cnt] = 0;
        sum_check += c[check_cnt];
    }
    else{
        sum_check -= c[check_cnt];
        if(avgV > MAX_VOLTAGE) c[check_cnt] = 1;
        else c[check_cnt] = 0;
        sum_check += c[check_cnt];
    }
    check_cnt++;
    if(!check_filled && check_cnt==NUM_CHECK) check_filled = true;
    if(check_cnt>=NUM_CHECK) check_cnt %= NUM_CHECK;

    if(sum_check >= EXCEED_THRESH * NUM_CHECK){
        if(dribblerSpeed == 0) dribblerSpeed = 0;
        else dribblerSpeed -= copysign(DEC_AMT, dribblerSpeed);
    }
    else{
        if(abs(dribblerSpeed) < dribbler_minspeed) dribblerSpeed = copysign(dribbler_minspeed, desiredSpeed);
        else{
            int diff = desiredSpeed - dribblerSpeed;
            dribblerSpeed += copysign(INC_AMT, diff);
            if((desiredSpeed >= 0 && dribblerSpeed > desiredSpeed) || (desiredSpeed < 0 && dribblerSpeed < desiredSpeed))
                dribblerSpeed = desiredSpeed;
        }
    }
}