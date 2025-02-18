#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>

class Kicker {
    public:
        Kicker(int pin) :
            _pin(pin) { // constructor
            pinMode(_pin, OUTPUT);
            digitalWrite(_pin, LOW);
            _lastKick = 0;
        };
        void kick() {
            float _curTime = millis();
            if(_curTime - _lastKick < 2000) return;
            digitalWrite(_pin, HIGH);
            delay(60);
            digitalWrite(_pin, LOW);
            _lastKick = millis();
        };

    private:
        const int _pin;
        float _lastKick;
};

#endif