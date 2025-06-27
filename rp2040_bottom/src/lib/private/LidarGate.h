#ifndef LIDAR_GATE_H
#define LIDAR_GATE_H

#include <Arduino.h>
#include "../public/analog_iic.h"
#include "../public/tofsense_f_iic.h"
#define THRESH_LEFT 0.20
#define THRESH_MID 0.28
#define THRESH_RIGHT 0.32

class LidarGate{
    public:
        LidarGate(uint8_t scl_pin, uint8_t sda_pin, uint8_t id) :
            _scl(scl_pin), _sda(sda_pin), _addr(id+0x08){
        } // constructor

        tofsense_f_output_parameter buffer;

        float readRaw(){
            IIC_Unpack_Dist(_scl, _sda, 0x24, 4, iic_read_buff, _addr, &buffer);
            return buffer.dis;
        }

        uint8_t checkBallCap(){
            float dist = readRaw();
            if(dist <= THRESH_LEFT) return (uint8_t)1;
            else if(dist <= THRESH_MID) return (uint8_t)2;
            else if(dist <= THRESH_RIGHT) return (uint8_t)3;
            else return (uint8_t)0; // not in ballcap
        }
    
    private:
        uint8_t _scl, _sda, _addr;
};

#endif