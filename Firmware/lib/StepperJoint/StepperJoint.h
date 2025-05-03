// StepperJoint.h

#ifndef STEPPERJOINT_H
#define STEPPERJOINT_H

#include <Arduino.h>
#include "EncoderLib.h"

class StepperJoint {
public:
    StepperJoint();

    void begin(uint8_t stepPin, uint8_t dirPin, EncoderLib* encoder, uint8_t ms1Pin, uint8_t ms2Pin);
    void setTarget(float angle);
    void setPIDGains(float kp, float ki, float kd);
    void setMicrostepping(int mode);
    void calibrateFromEncoder();
    void update();

    float getCurrentAngle() const;

    // PID setters/getters
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    float getKp() const;
    float getKi() const;
    float getKd() const;

    // ── New: deadband setter ────────────────────────────
    /// Only drive if |error| ≥ this many degrees
    void setDeadband(float degrees);

private:
    float angleDiff(float target, float current);

    uint8_t _stepPin, _dirPin, _ms1Pin, _ms2Pin;
    EncoderLib* _encoder;

    float _targetAngle, _currentAngle, _angleOffset;
    float _kp, _ki, _kd;
    float _lastError, _integral;

    int _microsteps;
    unsigned long _lastPIDTime, _lastStepTime;
    float _stepIntervalMicros;
    bool _stepState;

    // ── New member ─────────────────────────────────────
    float _deadbandDeg;  // if |error| < this, don’t step
};

#endif
