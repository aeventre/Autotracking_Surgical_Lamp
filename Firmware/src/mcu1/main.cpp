#include "CommandParser.h"
#include "EncoderLib.h"
#include "StepperJoint.h"
#include <Arduino.h>

StepperJoint joint1;
EncoderLib encoder1(Wire);

CommandParser parser;
CommandMessage desired;
CommandMessage current;

static constexpr uint8_t dirPin1 = 16;
static constexpr uint8_t stepPin1 = 17;

void setup()
{   
    Serial.begin(115200);
    parser.begin(Serial2);

    encoder1.begin(0, 1); // SDA = GPIO 0, SCL = GPIO 1
    joint1.begin(stepPin1, dirPin1, &encoder1);
    joint1.calibrateFromEncoder(); // Treat current encoder angle as 0°
    joint1.setPIDGains(5, 1, 0.5);
}

void loop()
{

    parser.readCommand(desired);
    joint1.update();



    if (parser.messageReceived())
    {
        joint1.setTarget(desired.a1);
        current.lightMode = desired.lightMode;
        current.a1 = joint1.getCurrentAngle();
        parser.sendStatus(current);
        parser.clearReceivedFlag();
    }
  }

// ====================================================================
// Test code for ServoJoint
// =========================

// #include "ServoJoint.h"
// #include <Adafruit_PWMServoDriver.h>
// #include <Arduino.h>
// #include <Wire.h>

// Adafruit_PWMServoDriver pca9685(0x40, Wire1);
// ServoJoint joint2;

// void setup()
// {
//     Serial.begin(115200);

//     Wire1.setSDA(2);
//     Wire1.setSCL(3);
//     Wire1.begin();

//     pca9685.begin();        // No args needed
//     pca9685.setPWMFreq(50); // 50Hz for servos

//     joint2.attach(0, &pca9685); // pass pointer here because ServoJoint expects it
//     joint2.setFeedbackType(ServoJoint::NONE);

//     Serial.println("ServoJoint Open Loop Test Ready");
// }

// void loop()
// {
// joint2.setTargetAngle(0);
// joint2.update();
// delay(1000);

// joint2.setTargetAngle(90);
// joint2.update();
// delay(1000);

// joint2.setTargetAngle(180);
// joint2.update();
// delay(1000);

//     pca9685.setPWM(0, 0, 150);
//     delay(1000); // 0 degrees
//     pca9685.setPWM(0, 0, 375);
//     delay(1000); // 90 degrees
//     pca9685.setPWM(0, 0, 600);
//     delay(1000); // 180 degrees
// }
