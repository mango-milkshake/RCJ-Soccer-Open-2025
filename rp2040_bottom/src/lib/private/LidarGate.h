#ifndef LIDAR_GATE_H
#define LIDAR_GATE_H

#include <Arduino.h>
#include "../public/analog_iic.h"
#include "../public/tofsense_f_iic.h"
#define LIDAR_BALLCAP_THRESH 0.04f

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

        bool checkBallCap(){
            return readRaw() <= LIDAR_BALLCAP_THRESH; 
            // if distance detected less than threshold, ball is in ballcap, return true
            // else ball is not in ballcap, return false

            // 0.16 to 0.20 is left, 0.20 to 0.28 is middle, 0.28 to 0.32 is right
        }
    
    private:
        uint8_t _scl, _sda, _addr;
};

#endif