#ifndef STEPPERJOINT_H
#define STEPPERJOINT_H

#include <Arduino.h>
#include "EncoderLib.h"

class StepperJoint
{
public:
    StepperJoint();

    void begin(uint8_t stepPin, uint8_t dirPin, EncoderLib* encoder, uint8_t ms1Pin, uint8_t ms2Pin);
    void setTarget(float angle);
    void setPIDGains(float kp, float ki, float kd);
    void setMicrostepping(int mode);
    void calibrateFromEncoder();
    void update();

    float getCurrentAngle();

private:
    float angleDiff(float target, float current);

    uint8_t _stepPin = 255;
    uint8_t _dirPin = 255;
    uint8_t _ms1Pin = 255;
    uint8_t _ms2Pin = 255;

    EncoderLib* _encoder = nullptr;

    float _targetAngle = 0.0f;
    float _currentAngle = 0.0f;
    float _angleOffset = 0.0f;

    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _lastError = 0.0f;
    float _integral = 0.0f;

    int _microsteps = 1;

    unsigned long _lastPIDTime = 0;
    unsigned long _lastStepTime = 0;
    float _stepIntervalMicros = 2000.0f;

    bool _stepState = false;
};

#endif
