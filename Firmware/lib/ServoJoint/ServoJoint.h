#ifndef SERVO_JOINT_H
#define SERVO_JOINT_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "EncoderLib.h"

class ServoJoint {
public:
    enum FeedbackType { NONE, ANALOG, ENCODER };

    ServoJoint();

    void attach(uint8_t pwmChannel, Adafruit_PWMServoDriver* driver);
    void setFeedbackType(FeedbackType type);
    void setAnalogPin(uint8_t pin);
    void setEncoder(EncoderLib* encoder);

    void setTargetAngle(float angle);
    void update();  // Call repeatedly in loop

    float getCurrentAngle() const;
    float getTargetAngle() const;

    void setPIDGains(float kp, float ki, float kd);

private:
    uint8_t _channel;
    Adafruit_PWMServoDriver* _driver;
    FeedbackType _fbType = NONE;

    float _targetAngle = 0.0f;
    float _currentAngle = 0.0f;
    float _lastError = 0.0f;
    float _integral = 0.0f;
    unsigned long _lastPIDTime = 0;

    float _kp = 5.0;
    float _ki = 0.1;
    float _kd = 0.5;

    uint8_t _analogPin = 255;     // Invalid until set
    EncoderLib* _encoder = nullptr;

    static constexpr int pulseMin = 150;
    static constexpr int pulseMax = 600;

    void writePWM(float angle); // sends to PCA9685
};

#endif
