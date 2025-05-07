// ServoJoint.h
#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include "EncoderLib.h"

class ServoJoint
{
public:
    enum FeedbackType { NONE, ANALOG, ENCODER };

    ServoJoint();

    void attach(uint8_t pwmChannel, Adafruit_PWMServoDriver* driver, bool reversed = false);
    void setFeedbackType(FeedbackType type);
    void setAnalogPin(uint8_t pin);
    void setEncoder(EncoderLib* encoder);
    void setTargetAngle(float angle);
    void setPIDGains(float kp, float ki, float kd);
    void setAngleRange(float rangeDegrees);
    void setPulseRange(uint16_t minPulse, uint16_t maxPulse);
    void setAnalogMapping(float slope, float intercept);
    void setAnalogLimits(int minADC, int maxADC);
    void setAngleOffset(float offset);

    float getCurrentAngle() const;
    float getTargetAngle() const;

    void update();

private:
    void writePWM(float angle);
    void resetFilter();

    uint8_t _channel = 0;
    Adafruit_PWMServoDriver* _driver = nullptr;

    FeedbackType _fbType = NONE;

    uint8_t _analogPin = 255;
    EncoderLib* _encoder = nullptr;

    float _targetAngle = 0.0f;
    float _currentAngle = 0.0f;

    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _angleRange = 180.0f;
    uint16_t _pulseMin = 102;
    uint16_t _pulseMax = 512;

    float _analogSlope = 0.0f;
    float _analogIntercept = 0.0f;
    int _analogMin = 0;
    int _analogMax = 4095;

    float _angleOffset = 0.0f;

    unsigned long _lastPIDTime = 0;
    float _lastError = 0.0f;
    float _integral = 0.0f;

    static constexpr int filterWindow = 10;
    int adc_samples[filterWindow] = {0};
    int adc_sum = 0;
    int sampleIndex = 0;

    bool _reverse = false;
};
