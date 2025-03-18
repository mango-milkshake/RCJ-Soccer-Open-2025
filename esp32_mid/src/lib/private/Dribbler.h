#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Arduino.h>
#include <SPI.h>
SPISettings MDSetting(100000, MSBFIRST, SPI_MODE1);

class MotorDriver {
    public:
        MotorDriver(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, uint8_t nsleep, uint8_t drvoff) :
            _mosi(mosi), _miso(miso), _sck(sck), _cs(cs), _nsleep(nsleep), _drvoff(drvoff) {
            pinMode(_cs, OUTPUT);
            pinMode(_nsleep, OUTPUT);
            pinMode(_drvoff, OUTPUT);
        }

        void init(){
            digitalWrite(_cs, HIGH);
            digitalWrite(_nsleep, HIGH);
            digitalWrite(_drvoff, LOW);
            SPI.begin(_sck, _miso, _mosi, _cs);
        }

        void sendCommand(uint8_t addr, uint8_t data){
            digitalWrite(_cs, LOW);
            SPI.beginTransaction(MDSetting);
            uint16_t value = (addr << 8) + data;
            uint16_t received = SPI.transfer16(value);
            Serial.println((uint8_t)(received), BIN);
            Serial.println((uint8_t)(received >> 8), BIN);
            digitalWrite(_cs, HIGH);
            SPI.endTransaction();
        }

        void setMode(){
            sendCommand(0b00001000, 0b10000000); // CLR_FLT comand

            // CONFIG2 Register
            // Bit 2-0: Set S_ITRIP = 0b001? to set V_itrip to 1.18V
            sendCommand(0b00001011, 0b00000100); // ITRIP

            // CONFIG1 Register
            sendCommand(0b00001010, 0b00010000);

            // CONFIG3 Register
            // Bits 1-0: Set S_MODE = 0b11 for PWM mode
            // Bits 4-2: Set S_SR = 0b111 for Slew Rate
            // Bits 7-6: Set TOFF = 0b01
            sendCommand(0b00001100, 0b01011111);
            digitalWrite(_drvoff, LOW);
        }

        void readRegister(uint8_t addr){
            sendCommand(addr, 0b00000000);
        }

        void clearFault(){
            sendCommand(0b00001000, 0b10000000);
        }

    private:
        const uint8_t _mosi, _miso, _sck, _cs, _nsleep, _drvoff;
};

#endif