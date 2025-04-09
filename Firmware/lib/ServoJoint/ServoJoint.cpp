#include "ServoJoint.h"

ServoJoint::ServoJoint() {}

void ServoJoint::attach(uint8_t pwmChannel, Adafruit_PWMServoDriver* driver)
{
    _channel = pwmChannel;
    _driver = driver;
}

void ServoJoint::setFeedbackType(FeedbackType type)
{
    _fbType = type;
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
    _targetAngle = constrain(angle, 0.0f, 180.0f);
}

void ServoJoint::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
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
    angle = constrain(angle, 0.0f, 180.0f);
    uint16_t pulse = map(angle, 0, 180, pulseMin, pulseMax);
    _driver->setPWM(_channel, 0, pulse);
}

void ServoJoint::update()
{
    if (_fbType == NONE)
    {
        writePWM(_targetAngle);
        return;
    }

    // --- Get current angle ---
    switch (_fbType)
    {
        case ANALOG:
            if (_analogPin != 255)
                _currentAngle = analogRead(_analogPin) * 270.0f / 4095.0f;
            break;
        case ENCODER:
            if (_encoder)
                _currentAngle = _encoder->getFilteredAngle();
            break;
        default:
            _currentAngle = 0;
    }

    // --- PID ---
    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f;
    if (dt <= 0.0f) return;
    _lastPIDTime = now;

    float error = _targetAngle - _currentAngle;
    _integral += error * dt;
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    float outputAngle = _kp * error + _ki * _integral + _kd * derivative;
    float commandedAngle = constrain(_currentAngle + outputAngle, 0.0f, 180.0f);

    writePWM(commandedAngle);
}
