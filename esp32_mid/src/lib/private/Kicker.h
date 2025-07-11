#ifndef KICKER_H
#define KICKER_H

#include <Arduino.h>
#define KICKER_DELAY 200
#define MIN_TIME_BETWEEN_KICKS 2000

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
            if(_curTime - _lastKick < MIN_TIME_BETWEEN_KICKS) return;
            digitalWrite(_pin, HIGH);
            delay(KICKER_DELAY);
            digitalWrite(_pin, LOW);
            _lastKick = millis();
        };

    private:
        const int _pin;
        float _lastKick;
};

#endif