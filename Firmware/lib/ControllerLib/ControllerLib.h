#ifndef CONTROLLERLIB_H
#define CONTROLLERLIB_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "EncoderLib.h"

#define dirPin 16
#define stepPin 17
#define neoPixelPin 18
#define laserPin 19
#define joint3FeedbackPin 26
#define joint4FeedbackPin 27

class ControllerLib
{
  public:
    ControllerLib();

    void begin();

    void receiveCommand(float *angle1, float *angle2, float *angle3, float *angle4, int *lightMode);

    void sendStatus(float *angle1, float *angle2, float *angle3, float *angle4, int *lightMode);

    void setServoAngles(float angle2, float angle3, float angle4);


  private:
    void getServoAngle(float *angle3, float *angle4);
    void updateJoint1PID();
    void updateJoint1Stepper();
    void setJoint2Target(float angle);
    float getJoint2Angle();

    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1); // Use I2C1
    EncoderLib encoder1;
    EncoderLib encoder2;
    


    // Keeps track of half duplex messaging, true if a completed message has been received
    // and false once a return message has been sent
    bool receiveState = false;

    // ===== Joint 1 PID control and stepper state =====
    float targetAngle1 = 0;
    float currentAngle1 = 0;
    float integral1 = 0;
    float lastError1 = 0;
    unsigned long lastPIDTime1 = 0;

    unsigned long lastStepTime1 = 0;
    float stepIntervalMicros1 = 1000000;
    int stepState1 = LOW;

    // PID gains
    float kp1 = 5.0;
    float ki1 = 0.1;
    float kd1 = 1.0;
};

#endif
