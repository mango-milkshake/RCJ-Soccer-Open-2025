#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Motor.h>
#include <Drive.h>

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
#define ACCEL_DELAY_MICROS 2000
#define ACCEL_DELAY_MILLIS 2

#define NUM_DRIVERS 4
const int num_bytes = 2*NUM_DRIVERS + 2;
uint8_t buffer[num_bytes];
uint8_t address[NUM_DRIVERS];
uint8_t data[NUM_DRIVERS];

uint8_t IN1_pin[NUM_DRIVERS] = {7, 8, 10, 13};
uint8_t IN2_pin[NUM_DRIVERS] = {6, 9, 11, 12};
uint8_t IPROPI_pin[NUM_DRIVERS] = {26, 27, 28, 29}; // make sure pins can read analog

#define speedMaximum 400000
SPISettings MDSetting(speedMaximum, MSBFIRST, SPI_MODE1);

uint8_t maxspeed = 50;

Motor motorFL(IN1_pin[0], IN2_pin[0], maxspeed, 0.9);
Motor motorFR(IN1_pin[3], IN2_pin[3], maxspeed, 0.9);
Motor motorBL(IN1_pin[1], IN2_pin[1], maxspeed, 1.0);
Motor motorBR(IN1_pin[2], IN2_pin[2], maxspeed, 1.0);

Drive bot(motorFR, motorBR, motorBL, motorFL);
float speed = 1.0;
float speedX, speedY, moveAngle;

void daisychain(uint8_t addr[], uint8_t d[]){
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(MDSetting);

  buffer[0] = 0b10000000 + NUM_DRIVERS; // Header 1
  buffer[1] = 0b10100000; // Header 2, change bit 5 to 1 for global CLR_FLT
  for (int i=2; i<NUM_DRIVERS+2; i++){
    // address bytes, last driver first
    buffer[i] = addr[i-2];
  }
  for (int i=NUM_DRIVERS+2; i<num_bytes; i++){
    buffer[i] = d[i-NUM_DRIVERS-2];
  }
  
  // for (int i=0; i<num_bytes; i++){
  //   SPI.transfer(&buffer[i], 1);
  // }
  SPI.transfer(buffer, num_bytes);

  for (int i=0; i<num_bytes; i++){
    Serial.print(buffer[i], BIN);
    Serial.print(" ");
  }
  Serial.println();

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("started");
  while(!Serial.available()) continue;
  while(Serial.available()) Serial.read();

  analogWriteFreq(5000);

  // Set pin modes
  Serial.println("Setting pin modes");
  pinMode(NSLEEP_PIN, OUTPUT);
  digitalWrite(NSLEEP_PIN, 1);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, 1);
  // pinMode(NFAULT_PIN, INPUT_PULLUP);
  pinMode(DRVOFF_PIN, OUTPUT);
  digitalWrite(DRVOFF_PIN, 0);

  for (int i=0; i<NUM_DRIVERS; i++){
    pinMode(IN1_pin[i], OUTPUT);
    pinMode(IN2_pin[i], OUTPUT);
    Serial.print(IN1_pin[i]);
    Serial.print(IN2_pin[i]);
  }
  Serial.println("Set pin modes");

  delay(5);

  // Set SPI
  Serial.println("Setting SPI");
  SPI.setRX(MISO_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setCS(CS_PIN);
  SPI.begin(); // initialize the SPI library

  // Set device configurations
  Serial.println("Sending commands");
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01001000;
    data[i] = 0b10000000; // Send CLR_FLT command
  }
  daisychain(address, data);
  delay(5);

  // CONFIG2 Register
  // Bit 2-0: Set S_ITRIP = 0b001? to set V_itrip to 1.18V
  Serial.println("Setting ITRIP");
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001011;
    data[i] = 0b00000100;
  }
  daisychain(address, data);
  delay(5);

  // CONFIG1 Register
  Serial.println("Setting config1");
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001010;
    data[i] = 0b00010000;
  }
  daisychain(address, data);
  delay(5);

  // CONFIG3 Register
  // Bits 1-0: Set S_MODE = 0b11 for PWM mode
  // Bits 4-2: Set S_SR = 0b111 for Slew Rate
  // Bits 7-6: Set TOFF = 0b11 for max TOFF (50 micros)
  Serial.println("Enter PWM mode");
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001100;
    data[i] = 0b01011111; // 0b11011111;
  }
  daisychain(address, data);
  digitalWrite(DRVOFF_PIN, 0);
  delay(5);

  strip.begin();
  strip.show();
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
  while(!Serial.available()) continue;

  Serial.print("FAULT: ");
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01000001;
    data[i] = 0b00000000;
  }
  daisychain(address, data);
  delay(5);
  Serial.println();

// move straight
//   bot.setDrive(speed, 0, 0); // speed, angle, rotationrate from pid calculations

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
//   for (int i=0; i<4; i++){
//     bot.setDrive(speed, 90*i, 0);
//     delay(300);
//   }

// move in circle
  for (int i=0; i<365; i++){
    bot.setDrive(speed, i, 0);
    delay(5);
  }

// testing
//   bot.setDrive(speed, 135, 0); // speed, angle, rotationrate from pid calculations

}