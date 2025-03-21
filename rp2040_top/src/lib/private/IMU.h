#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h"
#include <SPI.h>
#include <CommonUtils.h>

// SPISettings ICMSetting(1000000, MSBFIRST, SPI_MODE1);
#define AD0_VAL 1

class IMU {
    public:
        IMU(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, SPIClassRP2040 &spi) :
            _mosi(mosi), _miso(miso), _sck(sck), _cs(cs), _spi(spi) {  
        }

        ICM_20948_SPI ICM;
        icm_20948_DMP_data_t data;
        double yaw = 0, roll = 0, pitch = 0;
        double rawYaw = 0, rawRoll = 0, rawPitch = 0;
        double yawOffset = 0, rollOffset = 0, pitchOffset = 0;
        int16_t accelX = 0, accelY = 0, accelZ = 0;
        int16_t rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
        int16_t accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;

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
            success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
            success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
            success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
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

        bool readRawData(){
            bool status = _update();
            if(!status) return false;

            bool validData = true;
            if ((data.header & DMP_header_bitmap_Quat6) > 0){
                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                double qw = q0; 
                double qx = q2;
                double qy = q1;
                double qz = -q3;

                double t0 = +2.0 * (qw * qx + qy * qz);
                double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
                rawRoll = atan2(t0, t1) * 180.0 / PI;

                double t2 = +2.0 * (qw * qy - qx * qz);
                t2 = t2 > 1.0 ? 1.0 : t2;
                t2 = t2 < -1.0 ? -1.0 : t2;
                rawPitch = asin(t2) * 180.0 / PI;

                double t3 = +2.0 * (qw * qz + qx * qy);
                double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                rawYaw = atan2(t3, t4) * 180.0 / PI;
            }
            else validData = false;

            if ((data.header & DMP_header_bitmap_Accel) > 0){
                rawAccelX = data.Raw_Accel.Data.X;
                rawAccelY = data.Raw_Accel.Data.Y;
                rawAccelZ = data.Raw_Accel.Data.Z;
            }
            else validData = false;

            return validData;
        }

        void updateAllData(){
            bool status = readRawData();
            if(!status) return;

            yaw = rawYaw - yawOffset;
            LIM_ANGLE_180(yaw);
            pitch = rawPitch - pitchOffset;
            LIM_ANGLE_180(pitch);
            roll = rawRoll - rollOffset;
            LIM_ANGLE_180(roll);

            accelX = rawAccelX - accelXOffset;
            accelY = rawAccelY - accelYOffset;
            accelZ = rawAccelZ - accelZOffset;
        }

        void tareAll(){
            bool tared = false;
            while(!tared){
                bool status = readRawData();
                if(status) tared = true;
            }
            yawOffset = rawYaw;
            pitchOffset = rawPitch;
            rollOffset = rawRoll;
            
            accelXOffset = rawAccelX;
            accelYOffset = rawAccelY;
            accelZOffset = rawAccelZ;
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