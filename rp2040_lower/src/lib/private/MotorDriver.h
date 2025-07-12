#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#define NUM_DRIVERS 4
const int num_bytes = 2*NUM_DRIVERS + 2;
SPISettings MDSetting(400000, MSBFIRST, SPI_MODE1);

class MotorDriver {
    public:
        MotorDriver(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, uint8_t nsleep, uint8_t drvoff) :
            _mosi(mosi), _miso(miso), _sck(sck), _cs(cs), _nsleep(nsleep), _drvoff(drvoff) {
            pinMode(_cs, OUTPUT);
            pinMode(_nsleep, OUTPUT);
            pinMode(_drvoff, OUTPUT);
            analogWriteFreq(25000);
        }

        uint8_t address[NUM_DRIVERS], data[NUM_DRIVERS];
        uint8_t buffer[num_bytes];

        void init(){
            digitalWrite(_cs, HIGH);
            digitalWrite(_nsleep, HIGH);
            digitalWrite(_drvoff, LOW);

            SPI.setRX(_miso);
            SPI.setTX(_mosi);
            SPI.setSCK(_sck);
            SPI.setCS(_cs);
            SPI.begin();
        }

        void daisychain(uint8_t addr[], uint8_t d[]){
            digitalWrite(_cs, LOW);
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

            SPI.transfer(buffer, num_bytes);

            for (int i=0; i<num_bytes; i++){
                Serial.print(buffer[i], BIN);
                Serial.print(" ");
            }
            Serial.println();

            digitalWrite(_cs, HIGH);
            SPI.endTransaction();
        }

        void spiComms(uint8_t addr, uint8_t d){
            for (int i=0; i<NUM_DRIVERS; i++){
                // start from last driver
                address[i] = addr;
                data[i] = d;
            }
            daisychain(address, data);
        }

        void setMode(){
            spiComms(0b01001000, 0b10000000); // CLR_FLT comand

            // CONFIG2 Register
            // Bit 2-0: Set S_ITRIP = 0b001? to set V_itrip to 1.18V
            spiComms(0b00001011, 0b00000100); // ITRIP

            // CONFIG1 Register
            spiComms(0b00001010, 0b00010000);

            // CONFIG3 Register
            // Bits 1-0: Set S_MODE = 0b11 for PWM mode
            // Bits 4-2: Set S_SR = 0b111 for Slew Rate
            // Bits 7-6: Set TOFF = 0b01
            spiComms(0b00001100, 0b01011111);
            digitalWrite(_drvoff, LOW);
        }

        void readRegister(uint8_t addr){
            spiComms(addr, 0b00000000);
        }

    private:
        const uint8_t _mosi, _miso, _sck, _cs, _nsleep, _drvoff;
};

#endif