#ifndef LINE_H
#define LINE_H

#include <Arduino.h>
#define LINE_THRESH 900

class Line {
    public:
        Line(uint8_t selectPin0, uint8_t selectPin1, uint8_t selectPin2, uint8_t inputPin) :
            _selectPin{selectPin0, selectPin1, selectPin2}, _inputPin(inputPin) {
            for (int i=0; i<3; i++){
                pinMode(_selectPin[i], OUTPUT);
                digitalWrite(_selectPin[i], HIGH);
            }
            pinMode(_inputPin, INPUT);
        }

        void selectMuxPin(uint8_t pin){
            for (int i=0; i<3; i++){
                if(pin & (1<<i)) digitalWrite(_selectPin[i], HIGH);
                else digitalWrite(_selectPin[i], LOW);
            }
        }

        uint8_t readData(){
            uint8_t res = 0;
            for (uint8_t i=0; i<8; i++){
                selectMuxPin(i);
                int value = analogRead(_inputPin);
                if(value >= LINE_THRESH) res += (1 << i);
            }
            return res;
        }

    private:
        uint8_t _selectPin[3], _inputPin;
};

#endif