#include "StepperJoint.h"

StepperJoint::StepperJoint() {}

void StepperJoint::begin(uint8_t stepPin, uint8_t dirPin, EncoderLib* encoder)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _encoder = encoder;

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    _lastPIDTime = millis();
}

void StepperJoint::setTarget(float angle)
{
    _targetAngle = angle;
}

float StepperJoint::getCurrentAngle()
{
    return _currentAngle;
}

void StepperJoint::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void StepperJoint::update()
{
    // --- PID update (based on milliseconds) ---
    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0;
    if (dt <= 0.0) return;
    _lastPIDTime = now;

    _currentAngle = _encoder->getFilteredAngle();
    float error = _targetAngle - _currentAngle;
    _integral += error * dt;
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    float velocity = _kp * error + _ki * _integral + _kd * derivative;

    // Convert to steps/sec
    float stepsPerSec = abs(velocity) * microstepsPerDeg;

    if (stepsPerSec > 1.0) {
        _stepIntervalMicros = 1e6 / stepsPerSec;
        digitalWrite(_dirPin, velocity >= 0 ? HIGH : LOW);
    } else {
        _stepIntervalMicros = 1e9;  // effectively off
    }

    // --- Stepper pulse update (based on micros) ---
    if (_stepIntervalMicros < 1e6) {
        unsigned long nowMicros = micros();
        if (nowMicros - _lastStepTime >= _stepIntervalMicros) {
            _lastStepTime = nowMicros;
            _stepState = !_stepState;
            digitalWrite(_stepPin, _stepState);
        }
    }
}
