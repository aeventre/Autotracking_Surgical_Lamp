#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ServoJoint.h"
#include "StepperJoint.h"
#include "EncoderLib.h"
#include "CommandParser.h"

// --- PCA9685 Driver ---
Adafruit_PWMServoDriver pca9685(0x40, Wire1);

// --- Servo Joints ---
ServoJoint joint2;
ServoJoint joint3;
ServoJoint joint4;

// --- Stepper + Encoder ---
StepperJoint stepperJoint;
EncoderLib stepperEncoder(Wire);  // AS5600 encoder on Wire (default)

// --- Command Parser ---
CommandParser parser;
CommandMessage cmd;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire1.begin();
    pca9685.begin();
    pca9685.setPWMFreq(50);
    delay(10);

    stepperEncoder.begin(0, 1);
    delay(10);
    stepperEncoder.zero();
    Serial.println("Stepper encoder zeroed to current position.");

    stepperJoint.begin(17, 16, &stepperEncoder, 18, 19);
    stepperJoint.setMicrostepping(1);
    stepperJoint.setPIDGains(80.0f, 0.2f, 0.5f);
    stepperJoint.setTarget(stepperEncoder.getFilteredAngle());

    joint2.attach(0, &pca9685, true);
    joint2.setFeedbackType(ServoJoint::NONE);
    joint2.setAngleRange(180.0f);
    joint2.setPulseRange(102, 512);
    joint2.setTargetAngle(7.0f);

    joint3.attach(1, &pca9685);
    joint3.setFeedbackType(ServoJoint::ANALOG);
    joint3.setAnalogPin(A0);
    joint3.setAngleRange(270.0f);
    joint3.setPulseRange(102, 512);
    joint3.setAnalogMapping(0.413f, -7.57f);
    joint3.setAnalogLimits(30, 680);
    joint3.setAngleOffset(-7.0f);
    joint3.setPIDGains(2.0f, 0.0f, 0.1f);

    for (int i = 0; i < 10; ++i) {
        joint3.update();
        delay(10);
    }
    float stableStartAngle = joint3.getCurrentAngle();
    joint3.setTargetAngle(stableStartAngle);
    joint3.setPIDGains(1.0f, 0.2f, 0.2f);

    Serial.print("Servo 3 initialized at angle: ");
    Serial.println(stableStartAngle, 1);

    joint4.attach(2, &pca9685);
    joint4.setFeedbackType(ServoJoint::ANALOG);
    joint4.setAnalogPin(A1);
    joint4.setAngleRange(270.0f);
    joint4.setPulseRange(102, 512);
    joint4.setAnalogMapping(0.407f, -2.81f);
    joint4.setAnalogLimits(30, 675);
    joint4.setAngleOffset(-8.0f);
    joint4.setPIDGains(1.0f, 0.0f, 0.1f);

    for (int i = 0; i < 10; ++i) {
        joint4.update();
        delay(10);
    }
    float stableStartAngle2 = joint4.getCurrentAngle();
    joint4.setTargetAngle(stableStartAngle2);
    joint4.setPIDGains(4.0f, 0.3f, 0.2f);

    Serial.print("Servo 4 initialized at angle: ");
    Serial.println(stableStartAngle2, 1);

    parser.begin(Serial2); // RS485 Serial Port
    Serial.println("System initialized. Awaiting serial commands...");
}

void loop()
{
    joint2.update();
    joint3.update();
    joint4.update();
    stepperJoint.update();

    if (parser.readCommand(cmd))
    {
        joint2.setTargetAngle(cmd.a1);
        joint3.setTargetAngle(cmd.a2);
        joint4.setTargetAngle(cmd.a3);
        stepperJoint.setTarget(cmd.a4);
    }
<<<<<<< HEAD
  }
  
// ====================================================================
// Test code for ServoJoint
// =========================
=======

    CommandMessage status;
    status.a1 = joint2.getTargetAngle();
    status.a2 = joint3.getCurrentAngle();
    status.a3 = joint4.getCurrentAngle();
    status.a4 = stepperJoint.getCurrentAngle();
    status.lightMode = cmd.lightMode;
    parser.sendStatus(status);
>>>>>>> mcu1

    delay(20);
}
