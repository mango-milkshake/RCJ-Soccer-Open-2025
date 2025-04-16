/******************************************************************************
Mux_Analog_Input
SparkFun Multiplexer Analog Input Example
Jim Lindblom @ SparkFun Electronics
August 15, 2016
https://github.com/sparkfun/74HC4051_8-Channel_Mux_Breakout

This sketch demonstrates how to use the SparkFun Multiplexer
Breakout - 8 Channel (74HC4051) to read eight, separate
analog inputs, using just a single ADC channel.

Hardware Hookup:
Mux Breakout ----------- Arduino
     S0 ------------------- 2
     S1 ------------------- 3
     S2 ------------------- 4
     Z -------------------- A0
    VCC ------------------- 5V
    GND ------------------- GND
    (VEE should be connected to GND)

The multiplexers independent I/O (Y0-Y7) can each be wired
up to a potentiometer or any other analog signal-producing
component.

Development environment specifics:
Arduino 1.6.9
SparkFun Multiplexer Breakout - 8-Channel(74HC4051) v10
(https://www.sparkfun.com/products/13906)
******************************************************************************/
#include <Arduino.h>
/////////////////////
// Pin Definitions //
/////////////////////
#define NUM_MUX 4
const int selectPins[3] = {0, 1, 2}; // S0, S1, S2
int zin[NUM_MUX] = {26, 27, 28, 29};

// The selectMuxPin function sets the S0, S1, and S2 pins
// accordingly, given a pin from 0-7.
void selectMuxPin(byte pin)
{
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

void setup() 
{
  Serial.begin(115200); 
  // Set up the select pins as outputs:
  for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
    Serial.print(selectPins[i]);
  }
  for (int i=0; i<NUM_MUX; i++){
    pinMode(zin[i], INPUT);
  }
}

void loop() 
{
  // Loop through all eight pins.
  for (byte pin=0; pin<=7; pin++)
  {
    selectMuxPin(pin); 
    for (int i=0; i<NUM_MUX; i++){
      int value = analogRead(zin[i]);
      Serial.print(String(value) + " ");
    }
    Serial.print("|| ");
    if(pin==3 || pin==7) Serial.println();
  }
  Serial.println();
  delay(1000);
}
