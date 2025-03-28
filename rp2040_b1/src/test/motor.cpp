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
// #define NFAULT_PIN 7
#define DRVOFF_PIN 15
#define MOSI_PIN 3 // TX
#define MISO_PIN 0 // RX
#define SCK_PIN 2

#define TX_PIN 4
#define RX_PIN 5
#define DATA_LEN 3

byte buffer[DATA_LEN];

MotorDriver motor_driver(MOSI_PIN, MISO_PIN, SCK_PIN, CS_PIN, NSLEEP_PIN, DRVOFF_PIN);

uint8_t IN1_pin[NUM_DRIVERS] = {6, 9, 11, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {7, 8, 10, 12};
uint8_t NFAULT_pin[NUM_DRIVERS] = {26, 27, 28, 29};

uint8_t maxspeed = 50;

Motor motorFL(IN1_pin[0], IN2_pin[0], NFAULT_pin[0], maxspeed, 1.0);
Motor motorFR(IN1_pin[3], IN2_pin[3], NFAULT_pin[3], maxspeed, 1.0);
Motor motorBL(IN1_pin[1], IN2_pin[1], NFAULT_pin[1], maxspeed, 1.0);
Motor motorBR(IN1_pin[2], IN2_pin[2], NFAULT_pin[2], maxspeed, 1.0);
float lastFault = 0;

void setup()
{
  Serial.begin(115200);

  motor_driver.init();
  motor_driver.setMode();

  strip.begin();
  strip.setBrightness(brightness);
  strip.setPixelColor(0, strip.Color(0, 15, 0));
  strip.show();
  delay(1000);
}

void loop()
{
  Serial.println("looping");
  strip.setPixelColor(0, strip.Color(15, 15, 0));
  strip.show();
  
  motorFL.setSpeed(MOTOR_SPEED);
  motorFR.setSpeed(0.0);
  motorBL.setSpeed(0.0);
  motorBR.setSpeed(0.0);
  delay(1000);

  motorFL.setSpeed(0.0);
  motorFR.setSpeed(MOTOR_SPEED);
  motorBL.setSpeed(0.0);
  motorBR.setSpeed(0.0);
  delay(1000);

  motorFL.setSpeed(0.0);
  motorFR.setSpeed(0.0);
  motorBL.setSpeed(MOTOR_SPEED);
  motorBR.setSpeed(0.0);
  delay(1000);

  motorFL.setSpeed(0.0);
  motorFR.setSpeed(0.0);
  motorBL.setSpeed(0.0);
  motorBR.setSpeed(MOTOR_SPEED);
  delay(1000);

}
