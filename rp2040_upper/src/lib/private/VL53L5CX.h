#ifndef VL53L5CX_H
#define VL53L5CX_H

#include <Wire.h> 
#include <SparkFun_VL53L5CX_Library.h>
#define MAX_RETRY 3

class VL53L5CX {
    public:
        VL53L5CX(uint8_t scl, uint8_t sda, uint8_t lpin, uint8_t addr, int width, int freq, TwoWire &wire) :
            _scl(scl), _sda(sda), _lpin(lpin), _addr(addr), _width(width), _freq(freq), _wire(wire){
        }

        SparkFun_VL53L5CX sensor;
        VL53L5CX_ResultsData data;

        void initWire(){
            _wire.setSCL(_scl);
            _wire.setSDA(_sda);
            _wire.begin();
            _wire.setClock(400000);
        }

        void init(){
            digitalWrite(_lpin, HIGH);
            bool started = false;
            for (int i=0; i<MAX_RETRY; i++){
                if(!sensor.begin((byte)_addr, _wire)) {
                    Serial.print("Sensor not found on self address");
                    // Serial.println(sensor.getAddress());
                }
                else {
                    started = true;
                    break;
                }
            }
            if(!started){
                for (int i=0; i<MAX_RETRY; i++){
                    if(!sensor.begin((byte)41U, _wire)) {
                        Serial.print("Sensor not found on default address");
                    }
                    else {
                        started = true;
                        break;
                    }
                }
            }
            if(started){
                sensor.setAddress(_addr);
                sensor.setWireMaxPacketSize(128);
                sensor.setResolution(_width*_width);
                sensor.setRangingFrequency(_freq); // for 4x4, max 60; for 8x8, max 15
                sensor.startRanging();
            }
        }

        bool updateData(){
            // if data ready, read into data array
            if(sensor.isDataReady()){
                if(sensor.getRangingData(&data)) return true;
                else return false;
            }
            else return false;
        }

    private:
        const uint8_t _scl, _sda, _lpin, _addr;
        const int _width; // should only be 4 or 8
        const int _freq;
        TwoWire &_wire;
};

#endif