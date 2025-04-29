// ServoJoint.cpp
#include "ServoJoint.h"

ServoJoint::ServoJoint()
{
}

void ServoJoint::attach(uint8_t pwmChannel, Adafruit_PWMServoDriver* driver, bool reversed)
{
    _channel = pwmChannel;
    _driver = driver;
    _reverse = reversed;
}

void ServoJoint::setFeedbackType(FeedbackType type)
{
    _fbType = type;
    _lastPIDTime = millis();
    resetFilter();
}

void ServoJoint::setAnalogPin(uint8_t pin)
{
    _analogPin = pin;
    pinMode(pin, INPUT);
}

void ServoJoint::setEncoder(EncoderLib* encoder)
{
    _encoder = encoder;
}

void ServoJoint::setTargetAngle(float angle)
{
    _targetAngle = constrain(angle, 0.0f, _angleRange);
}

void ServoJoint::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void ServoJoint::setAngleRange(float rangeDegrees)
{
    _angleRange = constrain(rangeDegrees, 1.0f, 360.0f);
}

void ServoJoint::setPulseRange(uint16_t minPulse, uint16_t maxPulse)
{
    _pulseMin = minPulse;
    _pulseMax = maxPulse;
}

void ServoJoint::setAnalogMapping(float slope, float intercept)
{
    _analogSlope = slope;
    _analogIntercept = intercept;
}

void ServoJoint::setAnalogLimits(int minADC, int maxADC)
{
    _analogMin = minADC;
    _analogMax = maxADC;
}

void ServoJoint::setAngleOffset(float offset)
{
    _angleOffset = offset;
}

float ServoJoint::getCurrentAngle() const
{
    return _currentAngle;
}

float ServoJoint::getTargetAngle() const
{
    return _targetAngle;
}

void ServoJoint::writePWM(float angle)
{
    angle = constrain(angle, 0.0f, _angleRange);
    float normalized = angle / _angleRange;

    uint16_t pulse;
    if (_reverse)
    {
        pulse = _pulseMax - (_pulseMax - _pulseMin) * normalized;
    }
    else
    {
        pulse = _pulseMin + (_pulseMax - _pulseMin) * normalized;
    }

    pulse = constrain(pulse, _pulseMin, _pulseMax);

    if (_driver)
        _driver->setPWM(_channel, 0, pulse);
}

void ServoJoint::resetFilter()
{
    adc_sum = 0;
    sampleIndex = 0;
    for (int i = 0; i < filterWindow; ++i)
        adc_samples[i] = 0;
}

void ServoJoint::update()
{
    if (_fbType == NONE)
    {
        writePWM(_targetAngle);
        return;
    }

    int rawADC = 0;

    switch (_fbType)
    {
        case ANALOG:
            if (_analogPin != 255)
                rawADC = analogRead(_analogPin);
            break;
        case ENCODER:
            if (_encoder)
            {
                _currentAngle = _encoder->getFilteredAngle();
                break;
            }
            return;
        default:
            _currentAngle = 0;
            return;
    }

    int clampedADC = constrain(rawADC, _analogMin, _analogMax);
    float unfilteredAngle = (_analogSlope != 0.0f)
        ? (_analogSlope * clampedADC + _analogIntercept) + _angleOffset
        : 0.0f;

    if (_kp != 0.0f || _ki != 0.0f || _kd != 0.0f)
    {
        adc_sum -= adc_samples[sampleIndex];
        adc_samples[sampleIndex] = clampedADC;
        adc_sum += adc_samples[sampleIndex];
        sampleIndex = (sampleIndex + 1) % filterWindow;

        int filteredADC = adc_sum / filterWindow;
        _currentAngle = (_analogSlope != 0.0f)
            ? (_analogSlope * filteredADC + _analogIntercept) + _angleOffset
            : 0.0f;
    }
    else
    {
        _currentAngle = unfilteredAngle;
    }

    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f;
    if (dt <= 0.0f) return;
    _lastPIDTime = now;

    float error = _targetAngle - _currentAngle;
    _integral += error * dt;
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    float outputAngle = _kp * error + _ki * _integral + _kd * derivative;
    float commandedAngle = constrain(_currentAngle + outputAngle, 0.0f, _angleRange);

    writePWM(commandedAngle);
}
