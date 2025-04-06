#include "Controller.h"

Controller::Controller() : pwm(0x40, Wire1), encoder1(Wire), encoder2(Wire1)
{
}

void Controller::begin()
{
    // === Initialize Serial ===
    parser.begin(Serial2);

    // === Stepper Joint 1 ===
    encoder1.begin(0, 1); // Wire SDA/SCL
    joint1.begin(stepPin1, dirPin1, &encoder1);

    // === PCA9685 Servo Setup ===
    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire1.begin();
    pwm.begin();
    pwm.setPWMFreq(50);

    // === Servo Joint 2 (with encoder) ===
    encoder2.begin(4, 5);
    joint2.attach(0, &pwm);
    joint2.setFeedbackType(ServoJoint::ENCODER);
    joint2.setEncoder(&encoder2);

    // === Servo Joint 3 (analog feedback) ===
    joint3.attach(1, &pwm);
    joint3.setFeedbackType(ServoJoint::ANALOG);
    joint3.setAnalogPin(joint3FeedbackPin);

    // === Servo Joint 4 (analog feedback) ===
    joint4.attach(2, &pwm);
    joint4.setFeedbackType(ServoJoint::ANALOG);
    joint4.setAnalogPin(joint4FeedbackPin);

    // == = NeoPixel Lamp + Laser ===
    lamp.begin();
    lamp.show(); // Turn off initially

    pinMode(laserPin, OUTPUT);
    digitalWrite(laserPin, LOW);
}

void Controller::update()
{
    // 1. Read command
    if (parser.readCommand(desired))
    {
        joint1.setTarget(desired.a1);
        joint2.setTargetAngle(desired.a2);
        joint3.setTargetAngle(desired.a3);
        joint4.setTargetAngle(desired.a4);
        current.lightMode = desired.lightMode;
    }

    // 2. Update all joints
    joint1.update();
    joint2.update();
    joint3.update();
    joint4.update();

    // 3. Gather feedback
    current.a1 = joint1.getCurrentAngle();
    current.a2 = joint2.getCurrentAngle();
    current.a3 = joint3.getCurrentAngle();
    current.a4 = joint4.getCurrentAngle();

    // 4. Send status back
    parser.sendStatus(current);

}

void Controller::controlLamp(int lightMode)
{
    uint32_t color;

    switch (lightMode)
    {
    case 0: // OFF
        color = lamp.Color(0, 0, 0);
        digitalWrite(laserPin, LOW);
        break;

    case 1: // White
        color = lamp.Color(255, 255, 255);
        digitalWrite(laserPin, LOW);
        break;

    case 3: // Just Laser ON
        color = lamp.Color(0, 0, 0);
        digitalWrite(laserPin, HIGH);
        break;

    case 4: // Hot Pink + Laser ON
        color = lamp.Color(255, 105, 180);
        digitalWrite(laserPin, HIGH);
        break;

    case 5: // Banana Yellow
        color = lamp.Color(255, 255, 53);
        digitalWrite(laserPin, LOW);
        break;

    case 6: // White
        color = lamp.Color(255, 255, 255);
        digitalWrite(laserPin, LOW);
        break;

    default:
        color = lamp.Color(0, 0, 0);
        digitalWrite(laserPin, LOW);
        break;
    }

    // Apply color to all pixels in the ring
    for (int i = 0; i < lamp.numPixels(); i++)
    {
        lamp.setPixelColor(i, color);
    }

    lamp.show(); // Update the whole ring
}
