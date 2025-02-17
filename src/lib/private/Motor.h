#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    public:
        Motor(uint8_t pin1, uint8_t pin2, uint8_t maxspeed, float multiplier) :
            _pin1(pin1), _pin2(pin2), _maxspeed(maxspeed), _multiplier(multiplier) {
            pinMode(_pin1, OUTPUT);
            pinMode(_pin2, OUTPUT);
            // pin1 = clockwise, pin2 = anticlockwise
        }

        void setSpeed(float speed) {
            _speed = speed * _multiplier;
            _speed = constrain(_speed, -1, 1);
            if (_speed >= 0) {
                analogWrite(_pin2, 0);
                analogWrite(_pin1, abs(_speed)*_maxspeed);
            } else {
                analogWrite(_pin1, 0);
                analogWrite(_pin2, abs(_speed)*_maxspeed);
            }
            // Serial.print(_pin1);
            // Serial.print(": ");
            // Serial.print(speed);
            // Serial.println(abs(_speed)*_maxspeed);
        }

    private:
        const uint8_t _pin1, _pin2, _maxspeed;
        float _speed, _multiplier;
};

#endif