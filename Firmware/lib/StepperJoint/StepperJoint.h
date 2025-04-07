#ifndef STEPPERJOINT_H
#define STEPPERJOINT_H

#include <Arduino.h>
#include "EncoderLib.h"

class StepperJoint
{
public:
    StepperJoint();

    void begin(uint8_t stepPin, uint8_t dirPin, EncoderLib* encoder);
    void setTarget(float angle);
    void update();
    float getCurrentAngle();
    void setPIDGains(float kp, float ki, float kd);
    void calibrateFromEncoder();  // Treat current encoder angle as logical zero

private:
    float angleDiff(float target, float current);

    uint8_t _stepPin = 0;
    uint8_t _dirPin = 0;
    EncoderLib* _encoder = nullptr;

    float _targetAngle = 0.0f;
    float _currentAngle = 0.0f;

    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _integral = 0.0f;
    float _lastError = 0.0f;

    unsigned long _lastPIDTime = 0;
    unsigned long _lastStepTime = 0;
    float _stepIntervalMicros = 1000000.0f;
    bool _stepState = LOW;

    float _angleOffset = 0.0f;  // encoder offset to define logical zero
};

#endif
