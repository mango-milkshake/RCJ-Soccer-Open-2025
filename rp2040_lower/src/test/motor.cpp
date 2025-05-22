#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Motor.h>
#include <Drive.h>
#include <MotorDriver.h>

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

#define CLOCKWISE
#ifdef CLOCKWISE
#define MOTOR_SPEED 1.0
#else
#define MOTOR_SPEED -1.0
#endif

// PINS
#define CS_PIN 1
#define NSLEEP_PIN 14
#define DRVOFF_PIN 15
#define MOSI_PIN 3 // TX
#define MISO_PIN 0 // RX
#define SCK_PIN 2

#define TX_PIN 4
#define RX_PIN 5
#define DATA_LEN 3

byte buffer[DATA_LEN];

MotorDriver motor_driver(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

uint8_t IN1_pin[NUM_DRIVERS] = {7, 9, 11, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {6, 8, 10, 12};
uint8_t NFAULT_pin[NUM_DRIVERS] = {26, 27, 28, 29};

uint8_t maxspeed = 50;
std::vector<Motor> motors; // FL, BL, BR, FR

// Motor motorFL(IN1_pin[0], IN2_pin[0], NFAULT_pin[0], maxspeed, 1.0);
// Motor motorFR(IN1_pin[3], IN2_pin[3], NFAULT_pin[3], maxspeed, 1.0);
// Motor motorBL(IN1_pin[1], IN2_pin[1], NFAULT_pin[1], maxspeed, 1.0);
// Motor motorBR(IN1_pin[2], IN2_pin[2], NFAULT_pin[2], maxspeed, 1.0);
float lastFault = 0, lastSwap = 0;
int curidx = 0;

void checkFault(){
  bool faulted = false;
  for (int i=0; i<4; i++){
    if(digitalRead(motors[i].nfault)==LOW) faulted = true;
  }
  if(faulted){
      // Serial.println("faulted");
      float curTime = millis();
      if(curTime - lastFault >= 500){
          motor_driver.readRegister(0b01000001);
          lastFault = millis();
      }
  } 
}

void setup()
{
  Serial.begin(115200);

  for (int i=0; i<4; i++){
    motors.emplace_back(IN1_pin[i], IN2_pin[i], NFAULT_pin[i], maxspeed, 1.0);
  }
  motor_driver.init();
  motor_driver.setMode();

  strip.begin();
  strip.setBrightness(brightness);
  strip.setPixelColor(0, strip.Color(0, 15, 0));
  strip.show();
  delay(1000);
  lastSwap = millis();
}

void loop()
{
  Serial.println("looping");
  strip.setPixelColor(0, strip.Color(15, 15, 0));
  strip.show();

  checkFault();
  for (int i=0; i<4; i++){
    if(i==curidx) motors[i].setSpeed(MOTOR_SPEED);
    else motors[i].setSpeed(0.0);
    // delay(5);
  }
  if(millis()-lastSwap >= 1000){
    curidx++;
    curidx %= 4;
    lastSwap = millis();
  }
}
