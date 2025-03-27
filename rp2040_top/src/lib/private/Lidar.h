#ifndef LIDAR_H
#define LIDAR_H

#include "../public/analog_iic.h"
#include "../public/tofsense_f_iic.h"
#include <CommonUtils.h>
#include <RotatingCalipers.h>
#define MAX_LIDAR_DIST 3.2f

class Lidar{
    public:
        Lidar(uint8_t scl_pin, uint8_t sda_pin, uint8_t id, float angle, float calibration = 0) :
            _scl(scl_pin), _sda(sda_pin), _addr(id+0x08), _angle(angle), _calibration(calibration){
        } // constructor

        tofsense_f_output_parameter buffer;

        float readRaw(){
            IIC_Unpack_Dist(_scl, _sda, 0x24, 4, iic_read_buff, _addr, &buffer);
            // Serial.print(buffer.id);
            return buffer.dis;
        }

        Point readCoords(){
            // coordinates relative to bot
            Point p;
            float dist = readRaw() + _calibration;
            if(dist > MAX_LIDAR_DIST) dist = 0;
            p.x = cosf(RAD(_angle)) * dist;
            p.y = sinf(RAD(_angle)) * dist;
            return p;
        }
    
    private:
        uint8_t _scl, _sda, _addr;
        float _calibration, _angle;
};

#endif