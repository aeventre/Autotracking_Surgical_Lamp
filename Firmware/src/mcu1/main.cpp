// #include "Controller.h"
// #include <Arduino.h>

// Controller controller;

// float currentAngle1, currentAngle2, currentAngle3, currentAngle4, speed1;
// float desiredAngle1, desiredAngle2, desiredAngle3, desiredAngle4;
// int lightMode;

// void setup()
// {

// }

// void loop()
// {
// }

// =========================
// Test code for StepperJoint
// =========================

#include "EncoderLib.h"
#include "StepperJoint.h"
#include <Arduino.h>

StepperJoint joint1;
EncoderLib encoder1(Wire);  

static constexpr uint8_t dirPin1 = 16;
static constexpr uint8_t stepPin1 = 17;

void setup()
{
    Serial.begin(115200);
    Serial.println("Stepper Joint Test");

    encoder1.begin(0, 1); // SDA = GPIO 0, SCL = GPIO 1
    joint1.calibrateFromEncoder();   // Treat current encoder angle as 0°
    joint1.setPIDGains(10.0, 0.0, 0.5);
}

void loop()
{
    // for (int i = 0; i < 360; i += 10)
    // {
    //     joint1.setTarget(i);

    //     unsigned long start = millis();
    //     while (millis() - start < 1000)
    //     {
    //         joint1.update();
    //         Serial.print("Target: ");
    //         Serial.print(i);
    //         Serial.print(" | Current: ");
    //         Serial.println(joint1.getCurrentAngle());
    //     }
    // }

    joint1.setTarget(100.0);
    joint1.update();
    Serial.println(joint1.getCurrentAngle());
}
