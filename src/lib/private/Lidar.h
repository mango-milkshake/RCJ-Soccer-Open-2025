#ifndef LIDAR_H
#define LIDAR_H

#include "../public/analog_iic.h"
#include "../public/tofsense_f_iic.h"

class Lidar{
    public:
        Lidar(uint8_t scl_pin, uint8_t sda_pin, uint8_t id, float calibration = 0) :
            _scl(scl_pin), _sda(sda_pin), _addr(id+0x08), _calibration(calibration){
        } // constructor

        tofsense_f_output_parameter buffer;

        float readRaw(){
            IIC_Unpack_Dist(_scl, _sda, 0x24, 4, iic_read_buff, _addr, &buffer);
            // Serial.print(buffer.id);
            return buffer.dis;
        }
    
    private:
        uint8_t _scl, _sda, _addr;
        float _calibration;
};

#endif