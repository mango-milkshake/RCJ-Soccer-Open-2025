#include <Arduino.h>
#include <Wire.h>

#define SCL_PIN 7
#define SDA_PIN 6
TwoWire &_wire = Wire1;

void setup() {
    _wire.setSCL(SCL_PIN);
    _wire.setSDA(SDA_PIN);
    _wire.setClock(400000);
    //Wire.setTimeout(1); // set timeout to 1 ms
    _wire.begin();

    Serial.begin(115200);
    // while (!Serial);
    Serial.println("\nI2C Scanner");
}

void loop() {
    byte error, address;
    int  nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        _wire.beginTransmission(address);
        error = _wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

    delay(5000); // wait 5 seconds for next scan
}