/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to get all 64 pixels, at 15Hz, comma seperated output.
  This is handy for transmission to visualization programs such as Processing.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

long measurements = 0;         // Used to calculate actual output rate
long measurementStartTime = 0; // Used to calculate actual output rate

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire1.setSCL(7);
  Wire1.setSDA(6);
  Wire1.begin(); // This resets I2C bus to 100kHz
  Wire1.setClock(1000000); //Sensor has max I2C freq of 1MHz

  sensor.setWireMaxPacketSize(128); // Increase default from 32 bytes to 128 - not supported on all platforms

  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  while (sensor.begin((byte)41U, Wire1) == false)
  {
    Serial.println(F("Sensor not found - check your wiring."));
    delay(5);
    // while (1);
  }

  sensor.setResolution(8*8); // Enable all 64 pads

  imageResolution = sensor.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);         // Calculate printing width

  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  sensor.setRangingFrequency(15);

  sensor.startRanging();

  measurementStartTime = millis();
}

void loop()
{
  // Poll sensor for new data
  if (sensor.isDataReady() == true)
  {
    if (sensor.getRangingData(&measurementData)) // Read distance data into array
    {
      // The ST library returns the data transposed from zone mapping shown in datasheet
      // Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
      {
        for (int x = imageWidth - 1; x >= 0; x--)
        {
          Serial.print(measurementData.distance_mm[x + y]);
          Serial.print(",");
        }
      }
      Serial.println();

      // Uncomment to display actual measurement rate
      measurements++;
      float measurementTime = (millis() - measurementStartTime) / 1000.0;
      Serial.print("rate: ");
      Serial.print(measurements / measurementTime, 3);
      Serial.println("Hz");
    }
  }

  delay(5); // Small delay between polling
}