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

// PINS
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

uint8_t maxspeed = 20;

Motor motorFL(IN1_pin[0], IN2_pin[0], maxspeed, 1.0);
Motor motorFR(IN1_pin[3], IN2_pin[3], maxspeed, 1.0);
Motor motorBL(IN1_pin[1], IN2_pin[1], maxspeed, 1.0);
Motor motorBR(IN1_pin[2], IN2_pin[2], maxspeed, 1.0);

Drive bot(motorFR, motorBR, motorBL, motorFL);
float speed = 1.0;
float speedX, speedY, moveAngle;

int counter = 0;

void setup()
{
  Serial.begin(115200);
  // Serial.println("started");
  // while(!Serial.available()) continue;
  // while(Serial.available()) Serial.read();

  analogWriteFreq(5000);

  motor_driver.init();
  motor_driver.setMode();

  strip.begin();
  strip.setBrightness(brightness);
  strip.setPixelColor(0, strip.Color(0, 0, 15));
  strip.show();
  delay(1000);
}

void loop()
{
  Serial.println("looping");
  strip.setPixelColor(0, strip.Color(15, 15, 0));
  strip.show();
  // while(!Serial.available()) continue;

  // Serial.print("FAULT: ");
  // motor_driver.spiComms(0b01000001, 0b00000000);

// move straight
  // if(counter%500<250) bot.setDrive(speed, 0, 0);
  // else bot.setDrive(0, 0, 0);
  // counter++;
  // if(counter>=500) counter = 0;

// move in octagon
//   for (int i=0; i<8; i++){
//     bot.setDrive(speed, 45*i, 0);
//     delay(500);
//   }

// move in hexagon
//   for (int i=0; i<6; i++){
//     bot.setDrive(speed, 60*i+30, 0);
//     delay(300);
//   }

// move in square
  // for (int i=0; i<4; i++){
  //   bot.setDrive(speed, 90*i, 0);
  //   delay(300);
  // }

// move in circle
  // for (int i=0; i<365; i++){
  //   bot.setDrive(speed, i, 0);
  //   delay(5);
  // }

// testing
  bot.setDrive(speed, 0, 0);

}