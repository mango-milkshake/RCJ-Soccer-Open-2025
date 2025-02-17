#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <CommonUtils.h>
#include <Motor.h>

class Drive {
    public:
        Drive(Motor &motorFR, Motor &motorBR, Motor &motorBL, Motor &motorFL) :
            _motorFR(motorFR), _motorBR(motorBR), _motorBL(motorBL), _motorFL(motorFL) {
        }

        void setDrive(float speed, int angle, float rotationRate) {
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

            // Check if any of the wheel's speed exceeds ± 1,
            // if yes, find the wheel's speed that exceeded the most,
            // divide all the movement components by the amount needed such that it does not exceed ±1
            float maxSpeed    = max(abs(speedX), abs(speedY));
            float absRotation = abs(rotationRate);
            if (maxSpeed + absRotation > 1) {
                float k = (1 - absRotation) / maxSpeed;
                speedFR *= k;
                speedFL *= k;
                speedBR *= k;
                speedBR *= k;
            }

            speedFR += rotationRate;
            speedFL += rotationRate;
            speedBR += rotationRate;
            speedBR += rotationRate;

            _motorFL.setSpeed(speedFL + rotationRate);
            _motorFR.setSpeed(speedFR + rotationRate);
            _motorBL.setSpeed(speedBL + rotationRate);
            _motorBR.setSpeed(speedBR + rotationRate);
        };

    private:
        Motor _motorFL, _motorFR, _motorBL, _motorBR;
};

#endif