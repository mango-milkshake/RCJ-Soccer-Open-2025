#include <SPI.h>
#include <Arduino.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <queue>
#include <Adafruit_NeoPixel.h>

#define led_pin 16
#define led_count 1
#define brightness 50
Adafruit_NeoPixel strip(led_count, led_pin, NEO_GRB + NEO_KHZ800);

// SPI Protocol
#define SPI_ADDRESS_MASK 0x3F00 // Mask for SPI register address bits
#define SPI_ADDRESS_POS 8       // Position for SPI register address bits
#define SPI_DATA_MASK 0x00FF    // Mask for SPI register data bits
#define SPI_DATA_POS 0          // Position for SPI register data bits
#define SPI_RW_BIT_MASK 0x4000  // Mask for SPI register read write indication bit

// PINS
// Arduino CLK = PIN 13
// Arduino SDI (MISO) = PIN 12
// Arduino SDO (MOSI) = PIN 11
#define CS_PIN 5 // 1
#define NSLEEP_PIN 0 // 9
#define NFAULT_PIN 2 // 7
#define IN2_PIN 10 // 5
#define IN1_PIN 9 // 4
#define DRVOFF_PIN 1 // 6
#define MOSI_PIN 7 // 3 // TX
#define MISO_PIN 4 // 0 // RX
#define SCK_PIN 6 // 2
#define SW1_PIN 26
#define SW2_PIN 27
#define ACCEL_DELAY_MICROS 2000
#define ACCEL_DELAY_MILLIS 2

const uint8_t IPROPI_pin = 8;   // @jx put the IPROPI pin number here, also make sure its a pin that can read analog
const uint16_t A_IPROPI = 4750; // VQFN-HR package
const uint32_t IPROPI_resistance = 680;
const uint32_t ipropi_moving_average_size = 16384; // change this if uw, if its too big it wont fit

#define NUM_DRIVERS 3
const int num_bytes = 2*NUM_DRIVERS + 2;
uint8_t buffer[num_bytes];
uint8_t address[NUM_DRIVERS];
uint8_t data[NUM_DRIVERS];

#define speedMaximum 8000000
SPISettings MDSetting(speedMaximum, MSBFIRST, SPI_MODE1);

class MotorDriver
{
public:
  /**
   * This SPI function is used to write the set device configurations and operating
   * parameters of the device.
   * Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
   * Ax is address bit, Dx is data bits and R/W is read write bit.
   * For write R/W bit should 0.
   */
  void write8(uint8_t reg, uint8_t value)
  {
    volatile uint16_t reg_value = 0;                            // Variable for the combined register and data info
    reg_value |= ((reg << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); // Adding register address value
    reg_value |= ((value << SPI_DATA_POS) & SPI_DATA_MASK);     // Adding data value

    digitalWrite(CS_PIN, LOW);
    SPI.beginTransaction(MDSetting);

    // SPI.transfer((uint8_t)((reg_value >> 8) & 0xFF));
    // uint16_t received = SPI.transfer((uint8_t)(reg_value & 0xFF));
    uint16_t received16 = SPI.transfer16(reg_value & 0xFFFF);
    Serial.println((uint8_t)(received16), BIN);
    Serial.println((uint8_t)(received16 >> 8), BIN);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
  }

  void read8(uint8_t reg, uint8_t value)
  {
    volatile uint16_t reg_value = 0;                            // Variable for the combined register and data info
    reg_value |= ((reg << SPI_ADDRESS_POS) & 0x7F00); // Adding register address value
    reg_value |= ((value << SPI_DATA_POS) & SPI_DATA_MASK);     // Adding data value

    digitalWrite(CS_PIN, LOW);
    SPI.beginTransaction(MDSetting);

    // SPI.transfer((uint8_t)((reg_value >> 8) & 0xFF));
    // uint16_t recieved = SPI.transfer((uint8_t)(reg_value & 0xFF));
    uint16_t received16 = SPI.transfer16(reg_value & 0xFFFF);
    Serial.println((uint8_t)(received16), BIN);
    Serial.println((uint8_t)(received16 >> 8), BIN);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
  }

  void daisychain(uint8_t addr[], uint8_t d[]){
    digitalWrite(CS_PIN, LOW);
    SPI.beginTransaction(MDSetting);

    buffer[0] = 0b10000000 + NUM_DRIVERS; // Header 1
    buffer[1] = 0b10000000; // Header 2, change bit 5 to 1 for global CLR_FLT
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

  void accel(int pin){
    // Accelerate by increasing the PWM duty cycle
    for (int i=0; i<=255; i++){
      analogWrite(pin, i);
      delay(ACCEL_DELAY_MILLIS);
    }
  }

  void decel(int pin){
    // Decelerate by decreasing the PWM duty cycle
    for (int i=255; i>=0; i--)
    {
      analogWrite(pin, i);
      delay(ACCEL_DELAY_MILLIS);
    }
  }
};

MotorDriver motor_driver;
int counter = 0;
int delaytime = 200;
bool ran = false;

void setup()
{
  Serial.begin(115200);
  Serial.println("started");
  while(!Serial.available()){continue;}
  while(Serial.available()) Serial.read();

  // Set pin modes
  Serial.println("Setting pin modes");
  pinMode(NSLEEP_PIN, OUTPUT);
  digitalWrite(NSLEEP_PIN, 1);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, 1);
  pinMode(IN2_PIN, OUTPUT);
  // digitalWrite(IN2_PIN, 0);
  pinMode(IN1_PIN, OUTPUT);
  // digitalWrite(IN1_PIN, 0);
  pinMode(NFAULT_PIN, INPUT_PULLUP);
  digitalWrite(NFAULT_PIN, 0);
  pinMode(DRVOFF_PIN, OUTPUT);
  digitalWrite(DRVOFF_PIN, 0);
  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);
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
  // motor_driver.write8(0x08, 0b10000000); // Send CLR_FLT command

  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01001000;
    data[i] = 0b10000000;
  }
  motor_driver.daisychain(address, data);
  delay(5);

  // CONFIG4 Register
  // Bits 7-6: Set TOCP_SEL = 0b11 for lower TOCP
  // Bit 2: DRVOFF_SEL set to 1 by default
  Serial.println("Setting TOCP");
  // motor_driver.write8(0x0D, 0b11000100);

  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001101;
    data[i] = 0b11000100;
  }
  motor_driver.daisychain(address, data);
  delay(5);

  // CONFIG2 Register
  // Bit 2-0: Set S_ITRIP = 0b001 to set V_itrip to 1.18V
  Serial.println("Setting ITRIP");
  // motor_driver.write8(0x0B, 0b00000001);

  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001011;
    data[i] = 0b00000001;
  }
  motor_driver.daisychain(address, data);
  delay(5);

  // CONFIG3 Register
  // Bits 1-0: Set S_MODE = 0b11 for PWM mode
  // Bits 4-2: Set S_SR = 0b111 for Slew Rate
  // Bits 7-6: Set TOFF = 0b11 for max TOFF (50 micros)
  Serial.println("Enter PWM mode");
  // motor_driver.write8(0x0C, 0b11011111);

  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b00001100;
    data[i] = 0b11011111;
  }
  motor_driver.daisychain(address, data);
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
  float current, average;
  Serial.println("looping");
  strip.setPixelColor(0, strip.Color(15, 15, 0));
  strip.show();
  while(!Serial.available()) continue;

  // if(Serial.available()){
  //   while(Serial.available()) int key = Serial.read();
  //   counter++;
  //   delaytime -= 10;
  //   //if(delaytime<10) delaytime = 10;
  //   Serial.print("delay time");
  //   Serial.println(delaytime);
  // }

  // if(digitalRead(NFAULT_PIN)==LOW){
  //   Serial.print("FAULT");
  //   strip.setPixelColor(0, strip.Color(0, 15, 0));
  //   strip.show();
  // }

  Serial.print("CONFIG2: ");
  // motor_driver.read8(0b01001011, 0b00000000);
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01001011;
    data[i] = 0b00000000;
  }
  motor_driver.daisychain(address, data);
  delay(5);
  Serial.println();

  Serial.print("CONFIG3: ");
  // motor_driver.read8(0b01001100, 0b00000000);
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01001100;
    data[i] = 0b00000000;
  }
  motor_driver.daisychain(address, data);
  delay(5);
  Serial.println();

  Serial.print("CONFIG4: ");
  // motor_driver.read8(0b01001101, 0b00000000);
  for (int i=0; i<NUM_DRIVERS; i++){
    // start from last driver
    address[i] = 0b01001101;
    data[i] = 0b00000000;
  }
  motor_driver.daisychain(address, data);
  delay(5);
  Serial.println();
  
  // if(!ran){
  // // analogWrite(IN2_PIN, 255);
  // // delay(2);
  // for (int i = 0; i <= 255; i++)
  // {
  //   analogWrite(IN2_PIN, i);
  //   delayMicroseconds(2000);
  //   // current = motor_driver.read_current();
  //   // average = motor_driver.motor_current.add(current);
  // }
  // delay(2);
  // analogWrite(IN2_PIN, 0);
  // delay(2);
  // ran = true;
  // }

  // if(digitalRead(SW1_PIN)==HIGH){
  //   motor_driver.accel(IN1_PIN);
  //   motor_driver.decel(IN1_PIN);
  // }
  // else if(digitalRead(SW2_PIN)==HIGH){
  //   motor_driver.accel(IN2_PIN);
  //   motor_driver.decel(IN2_PIN);
  // }

}