#ifndef PID_H
#define PID_H

#define DECAY 0.99
#define MAX_I 0.3

#include <Arduino.h>
class PID {
    public:
        PID(float kp, float ki, float kd, float timeStep) :
            _timeStep(timeStep), _kp(kp), _ki(ki), _kd(kd) {
        }

        float compute(float goal, float actual) {
            unsigned long now = micros();
            if (now - _lastTime > _timeStep) {
                double dt = (now - _lastTime) / 1000;
                _lastTime  = now;

                double error = actual - goal; // Floats are not precise enough

                float output = 0;

                if (_kp) { // Proportional component
                    output += _kp * error;
                };

                if (_ki) { // Integral component
                    _integral += error * dt;
                    _integral *= DECAY;
                    double a = _ki * _integral;
                    output += constrain(a, -MAX_I, MAX_I);
                };

                if (_kd) { // Derivative component
                    if(error == _lastError){
                        output += _lastD;
                    }
                    else {
                        double ddt = (now - _lastDTime) / 1000;
                        double curD = _kd * (error - _lastError) / ddt;
                        output += curD;
                        _lastDTime = now;
                        _lastD = curD;
                    }
                    _lastError = error;
                };

                _lastOutput = output;
                return output; // constrain(output, -1, 1)

            } else
                return _lastOutput;
        } 

        void setConfig(float kp, float ki, float kd) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
        };

        void reset() {
            _integral = 0;
            _lastError = 0;
            _lastOutput = 0;
            _lastD = 0;
        }

    private:
        float _kp, _ki, _kd, _integral;
        double _lastError, _lastD;
        unsigned long _lastTime, _lastDTime;
        float _lastOutput;
        const float _timeStep;
};

#endif