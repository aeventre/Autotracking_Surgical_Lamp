#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>

#include "CommandParser.h"
#include "StepperJoint.h"
#include "ServoJoint.h"
#include "EncoderLib.h"

#define lampPin 18
#define laserPin 19

class Controller {
public:
    Controller();

    void begin();
    void update();  // Call this in loop()

private:

    void controlLamp(int mode);
    // RS-485 communication
    CommandParser parser;
    CommandMessage desired;
    CommandMessage current;

    // Stepper joint (Joint 1)
    StepperJoint joint1;
    EncoderLib encoder1;

    // Servo joint (Joint 2 with encoder)
    ServoJoint joint2;
    EncoderLib encoder2;

    // Servo joints (Joint 3 & 4 with analog feedback)
    ServoJoint joint3;
    ServoJoint joint4;

    // PWM servo driver (PCA9685)
    Adafruit_PWMServoDriver pwm;
    Adafruit_NeoPixel lamp = Adafruit_NeoPixel(1, lampPin, NEO_GRB + NEO_KHZ800);

    // Pin definitions
    static constexpr uint8_t dirPin1 = 16;
    static constexpr uint8_t stepPin1 = 17;
    static constexpr uint8_t joint3FeedbackPin = 26;
    static constexpr uint8_t joint4FeedbackPin = 27;
};

#endif
