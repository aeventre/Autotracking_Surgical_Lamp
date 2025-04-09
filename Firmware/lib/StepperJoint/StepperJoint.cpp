#include "StepperJoint.h"
#include <Arduino.h>

StepperJoint::StepperJoint()
{
}

void StepperJoint::begin(uint8_t stepPin, uint8_t dirPin, EncoderLib *encoder)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _encoder = encoder;

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    _lastPIDTime = millis();
    _lastStepTime = micros();
}

void StepperJoint::calibrateFromEncoder()
{
    _angleOffset = fmod(_encoder->getFilteredAngle(), 360.0f);
}

void StepperJoint::setTarget(float angle)
{
    _targetAngle = fmod(angle + 360.0f, 360.0f);
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
    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f;
    if (dt <= 0.0f)
        return;
    _lastPIDTime = now;

    float rawAngle = _encoder->getFilteredAngle();

    // Apply calibration offset and wrap to [0, 360)
    float logicalAngle = fmod((rawAngle - _angleOffset + 360.0f), 360.0f);
    _currentAngle = logicalAngle;

    float error = angleDiff(_targetAngle, _currentAngle);
    _integral += error * dt;
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    float velocity = _kp * error + _ki * _integral + _kd * derivative;

    static constexpr float microstepsPerDeg = (200.0f * 16.0f) / 360.0f;
    float stepsPerSecond = max(abs(velocity) * microstepsPerDeg, 0.5f);
    _stepIntervalMicros = 1e6 / stepsPerSecond;

    digitalWrite(_dirPin, velocity >= 0 ? LOW : HIGH);

    unsigned long nowMicros = micros();
    if (nowMicros - _lastStepTime >= _stepIntervalMicros)
    {
        _lastStepTime = nowMicros;
        _stepState = !_stepState;
        digitalWrite(_stepPin, _stepState);
    }

    // Optional debug
    // Serial.print("Target: "); Serial.print(_targetAngle);
    // Serial.print(" | Current: "); Serial.print(_currentAngle);
    // Serial.print(" | Error: "); Serial.println(error);
}

float StepperJoint::angleDiff(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return diff;
}
