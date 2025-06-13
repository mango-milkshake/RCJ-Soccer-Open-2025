#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <CommonUtils.h>
#include <Motor.h>

class Drive {
    public:
        Drive(Motor &motorFR, Motor &motorBR, Motor &motorBL, Motor &motorFL, int maxspeed) :
            _motorFR(motorFR), _motorBR(motorBR), _motorBL(motorBL), _motorFL(motorFL), _maxspeed(maxspeed) {
        }

        void setDriveOld(float speed, int angle, float rotationRate) {
            rotationRate = constrain(rotationRate, -1, 1);
            speed        = constrain(speed, -1, 1);
            // when rotationRate == 0, bot moves straight
            // when rotationRate == 1, bot rotates clockwise on the spot

            float speedX, speedY;
            speedX = speed * cosf(RAD(angle + 45));
            speedY = speed * sinf(RAD(angle + 45));

            float speedFL, speedFR, speedBL, speedBR; 
            speedFR = speedX;
            speedBR = speedY;
            speedFL = -speedY;
            speedBL = -speedX;

            float maxSpeed    = max(abs(speedX), abs(speedY));
            if (maxSpeed > 1) {
                float k = 1 / maxSpeed;
                speedFR *= k;
                speedFL *= k;
                speedBR *= k;
                speedBL *= k;
            }

            speedFR = 0.8 * speedFR - 0.2 * rotationRate;
            speedFL = 0.8 * speedFL - 0.2 * rotationRate;
            speedBL = 0.8 * speedBL - 0.2 * rotationRate;
            speedBR = 0.8 * speedBR - 0.2 * rotationRate;

            _motorFL.setSpeed(speedFL);
            _motorFR.setSpeed(speedFR);
            _motorBL.setSpeed(speedBL);
            _motorBR.setSpeed(speedBR);

            Serial.print(speedFR);
            Serial.print("\t");
            Serial.print(speedFL);
            Serial.print("\t");
            Serial.print(speedBL);
            Serial.print("\t");
            Serial.print(speedBR);
            Serial.println();
        };

        void setDrive(int speedX, int speedY, int rotationRate) {
            // DEBUG(speedX);
            // DEBUG(speedY);
            // DEBUG(rotationRate);

            float speedFL, speedFR, speedBL, speedBR; 
            speedFR = speedX + rotationRate;
            speedBR = speedY + rotationRate;
            speedFL = (-speedY) + rotationRate;
            speedBL = (-speedX) + rotationRate;
            // speedFR = 0.6 * speedX + 0.4 * rotationRate;
            // speedBR = 0.6 * speedY + 0.4 * rotationRate;
            // speedFL = 0.6 * (-speedY) + 0.4 * rotationRate;
            // speedBL = 0.6 * (-speedX) + 0.4 * rotationRate;

            float maxSpeed = max(max(abs(speedFR), abs(speedFL)), max(abs(speedBR), abs(speedBL)));
            // DEBUG(maxSpeed);
            if (maxSpeed > _maxspeed) {
                float k = _maxspeed / maxSpeed;
                speedFR *= k;
                speedFL *= k;
                speedBR *= k;
                speedBL *= k;
            }

            _motorFL.setSpeed(speedFL);
            _motorFR.setSpeed(speedFR);
            _motorBL.setSpeed(speedBL);
            _motorBR.setSpeed(speedBR);

            // Serial.print(speedFR);
            // Serial.print("\t");
            // Serial.print(speedFL);
            // Serial.print("\t");
            // Serial.print(speedBL);
            // Serial.print("\t");
            // Serial.print(speedBR);
            // Serial.println(" // FR FL BL BR");
        };

    private:
        Motor _motorFL, _motorFR, _motorBL, _motorBR;
        int _maxspeed;
};

#endif