#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h"
#include <SPI.h>

// SPISettings ICMSetting(1000000, MSBFIRST, SPI_MODE1);
#define AD0_VAL 1

class IMU {
    public:
        IMU(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, SPIClassRP2040 &spi) :
            _mosi(mosi), _miso(miso), _sck(sck), _cs(cs), _spi(spi) {  
        }

        ICM_20948_SPI ICM;
        icm_20948_DMP_data_t data;
        double offset = 0, tareVal = 631, lastYaw = 631;

        void init(){
            _spi.setRX(_miso);
            _spi.setTX(_mosi);
            _spi.setSCK(_sck);
            _spi.setCS(_cs);
            // _spi.begin(_sck, _miso, _mosi, _cs);
            _spi.begin();

            Serial.println("Initialised SPI");

            ICM.enableDebugging();

            Serial.println("Enabled ICM Debugging");

            bool initialized = false;
            while(!initialized){
                ICM.begin(_cs, _spi);
                Serial.print(F("Initialization of the sensor returned: "));
                Serial.println(ICM.statusString());
                if (ICM.status != ICM_20948_Stat_Ok) Serial.println(F("Trying again,,,"));
                else initialized = true;
            }
            Serial.println(F("Device connected!"));

            bool success = true;
            success &= (ICM.initializeDMP() == ICM_20948_Stat_Ok);
            success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
            // success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
            success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
            // success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
            success &= (ICM.enableFIFO() == ICM_20948_Stat_Ok); // enable FIFO
            success &= (ICM.enableDMP() == ICM_20948_Stat_Ok); // enable DMP
            success &= (ICM.resetDMP() == ICM_20948_Stat_Ok); // reset DMP
            success &= (ICM.resetFIFO() == ICM_20948_Stat_Ok); // reset FIFO

            if(success) Serial.println(F("DMP enabled"));
            else {
                Serial.println(F("Failed to enable DMP"));
                while(1) ;
            }
        }

        double readYaw(){
            bool status = _update();
            if (!status){
                return lastYaw;
            }
            if ((data.header & DMP_header_bitmap_Quat6) > 0){
                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                double qw = q0; 
                double qx = q2;
                double qy = q1;
                double qz = -q3;

                double t3 = +2.0 * (qw * qz + qx * qy);
                double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                double yaw = atan2(t3, t4) * 180.0 / PI;
                
                yaw = yaw - offset;
                if(yaw < -180) yaw += 180.0;
                else if(yaw > 180) yaw -= 180.0;
                lastYaw = yaw;
                return yaw;
            }
            else return lastYaw;
        }

        int16_t readAccelX(){
            bool status = _update();
            if (!status){
                return 631;
            }
            if ((data.header & DMP_header_bitmap_Accel) > 0){
                int16_t accel_x = data.Raw_Accel.Data.X;
                // int16_t accel_y = data.Raw_Accel.Data.Y;
                // int16_t accel_z = data.Raw_Accel.Data.Z;
                return accel_x;
            }
            else return 267;
        }
        void tareYaw(){
            while(tareVal==631 || tareVal==267){
                tareVal = readYaw();
            }
            offset = tareVal;
        }

    private:
        const int _mosi, _miso, _sck, _cs, _dataSize = 16;
        SPIClassRP2040 &_spi;

        bool _update(){
            uint16_t fifoCount;
            ICM.getFIFOcount(&fifoCount);
            if (fifoCount < _dataSize) {
                return false; 
            }
            ICM.readDMPdataFromFIFO(&data);
            return ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail));
        }
};

#endif