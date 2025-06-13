#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#define MAX_CHANGE 10

class Motor {
    public:
        const uint8_t nfault;

        Motor(uint8_t pin1, uint8_t pin2, uint8_t nfault_pin, uint8_t maxspeed, float multiplier) :
            _pin1(pin1), _pin2(pin2), nfault(nfault_pin), _maxspeed(maxspeed), _multiplier(multiplier) {
            pinMode(_pin1, OUTPUT);
            pinMode(_pin2, OUTPUT);
            pinMode(nfault, INPUT_PULLUP);
            // pin1 = clockwise, pin2 = anticlockwise
        }

        void setSpeed(float speed) {
            _speed = speed * _multiplier;
            // _speed = constrain(_speed, -1, 1);
            // _speedToSet = _speed * _maxspeed;
            _speedToSet = constrain(_speed, -_maxspeed, _maxspeed);
            if(abs(_speedToSet) > 2){
                _speedToSet += copysign(27, _speedToSet);
            }
            _speedToSet = constrain(_speedToSet, _lastSpeed-MAX_CHANGE, _lastSpeed+MAX_CHANGE);
            if (_speedToSet > 0) {
                analogWrite(_pin2, 0);
                analogWrite(_pin1, abs(_speedToSet));
            } 
            else {
                analogWrite(_pin1, 0);
                analogWrite(_pin2, abs(_speedToSet));
            }
            _lastSpeed = _speedToSet;
        }

    private:
        const uint8_t _pin1, _pin2, _maxspeed;
        float _speed, _multiplier, _lastSpeed = 0, _speedToSet;
};

#endif