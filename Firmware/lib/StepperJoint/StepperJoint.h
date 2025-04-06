#ifndef STEPPER_JOINT_H
#define STEPPER_JOINT_H

#include "EncoderLib.h"
#include <Arduino.h>

class StepperJoint
{
  public:
    StepperJoint();

    void begin(uint8_t stepPin, uint8_t dirPin, EncoderLib *encoder);
    void setTarget(float angle);
    float getCurrentAngle();
    void update(); // Call repeatedly in loop()

    void setPIDGains(float kp, float ki, float kd);

  private:
    uint8_t _stepPin;
    uint8_t _dirPin;
    EncoderLib *_encoder;

    // PID state
    float _targetAngle = 0;
    float _currentAngle = 0;
    float _lastError = 0;
    float _integral = 0;
    unsigned long _lastPIDTime = 0;

    // PID gains
    float _kp = 5.0;
    float _ki = 0.1;
    float _kd = 1.0;

    // Stepping state
    float _stepIntervalMicros = 1e6;
    int _stepState = LOW;
    unsigned long _lastStepTime = 0;

    static constexpr float microstepsPerDeg = (200.0f * 16.0f) / 360.0f; // Assuming 1/16 microstepping
};

#endif
