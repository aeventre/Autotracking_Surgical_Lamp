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
EncoderLib stepperEncoder(Wire); // AS5600 encoder on Wire (default)

// --- RS‑485 Command Parser ---
CommandParser rs485;
CommandMessage cmdMsg;

void setup()
{
    // --- USB‑Serial for debug ---
    Serial.begin(115200);
    delay(2000);

    // --- RS‑485 on Serial2 (Pico TX=4, RX=5) ---
    rs485.begin(Serial2);

    // --- I2C and PCA9685 ---
    Wire1.setSDA(2);
    Wire1.setSCL(3);
    Wire1.begin();
    pca9685.begin();
    pca9685.setPWMFreq(50);
    delay(10);

    // --- Stepper Encoder (AS5600) ---
    stepperEncoder.begin(0, 1);
    delay(10);
    stepperEncoder.zero();
    Serial.println("Stepper encoder zeroed to current position.");

    // --- StepperJoint Setup ---
    stepperJoint.begin(17, 16, &stepperEncoder, 18, 19); // step, dir, encoder, ms1, ms2
    stepperJoint.setMicrostepping(1);                    // full stepping
    stepperJoint.setPIDGains(80.0f, 0.2f, 0.5f);         // initial PID
    stepperJoint.setTarget(stepperEncoder.getFilteredAngle());

    // --- Joint 2: Open Loop Servo ---
    joint2.attach(0, &pca9685, true);
    joint2.setFeedbackType(ServoJoint::NONE);
    joint2.setAngleRange(180.0f);
    joint2.setPulseRange(102, 512);
    joint2.setTargetAngle(7.0f);

    // --- Joint 3: Analog PID Servo ---
    joint3.attach(1, &pca9685);
    joint3.setFeedbackType(ServoJoint::ANALOG);
    joint3.setAnalogPin(A0);
    joint3.setAngleRange(270.0f);
    joint3.setPulseRange(102, 512);
    joint3.setAnalogMapping(0.413f, -7.57f);
    joint3.setAnalogLimits(30, 680);
    joint3.setAngleOffset(-7.0f);
    joint3.setPIDGains(2.0f, 0.0f, 0.1f);
    // stabilize filter
    for (int i = 0; i < 10; ++i)
    {
        joint3.update();
        delay(10);
    }
    float stable3 = joint3.getCurrentAngle();
    joint3.setTargetAngle(stable3);
    joint3.setPIDGains(1.0f, 0.2f, 0.2f);
    Serial.print("Servo 3 initialized at angle: ");
    Serial.println(stable3, 1);

    // --- Joint 4: Analog PID Servo ---
    joint4.attach(2, &pca9685);
    joint4.setFeedbackType(ServoJoint::ANALOG);
    joint4.setAnalogPin(A1);
    joint4.setAngleRange(270.0f);
    joint4.setPulseRange(102, 512);
    joint4.setAnalogMapping(0.407f, -2.81f);
    joint4.setAnalogLimits(30, 675);
    joint4.setAngleOffset(-7.0f);
    joint4.setPIDGains(1.0f, 0.0f, 0.1f);
    // stabilize filter
    for (int i = 0; i < 10; ++i)
    {
        joint4.update();
        delay(10);
    }
    float stable4 = joint4.getCurrentAngle();
    joint4.setTargetAngle(stable4);
    joint4.setPIDGains(5.0f, 0.2f, 0.1f);
    Serial.print("Servo 4 initialized at angle: ");
    Serial.println(stable4, 1);

    Serial.println("System initialized. Listening for RS‑485 commands.");
}

void loop()
{
    // --- Process incoming RS‑485 frame ---
    if (rs485.readCommand(cmdMsg))
    {
        // Map fields to targets
        stepperJoint.setTarget(cmdMsg.a1);
        joint2.setTargetAngle(-1.0*cmdMsg.a2);
        joint3.setTargetAngle(cmdMsg.a3);
        joint4.setTargetAngle(cmdMsg.a4);
        rs485.clearReceivedFlag();
        rs485.sendStatus(cmdMsg);
    }

    // --- Update all joints ---
    joint2.update();
    joint3.update();
    joint4.update();
    stepperJoint.update();


    // --- Debug print to USB‑Serial ---
    // Serial.print("Stepper → ");
    // Serial.print(cmdMsg.a1, 1);
    // Serial.print(" | J2 → ");
    // Serial.print(cmdMsg.a2, 1);
    // Serial.print(" | J3 → ");
    // Serial.print(cmdMsg.a3, 1);
    // Serial.print(" | J4 → ");
    // Serial.println(cmdMsg.a4, 1);
}

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

// #include "CommandParser.h"
// #include "EncoderLib.h"
// #include "StepperJoint.h"
// #include "ServoJoint.h"
// #include <Adafruit_NeoPixel.h>
// #include <Adafruit_PWMServoDriver.h>
// #include <Arduino.h>

// // --- Objects ---
// StepperJoint joint1;
// EncoderLib encoder1(Wire);
// ServoJoint servo1, servo2, servo3;
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// CommandParser parser;
// CommandMessage desired;
// CommandMessage current;

// void handleLightMode(int mode);
// void laserOn();
// void laserOff();
// void laserBlink();
// void pixelsOff();
// void pixelsWhite();
// void pixelLightShow();
// void rainbowCycle(uint8_t wait);

// // --- Constants ---
// static constexpr uint8_t dirPin1 = 16;
// static constexpr uint8_t stepPin1 = 17;
// static constexpr uint8_t laserPin = 21;
// static constexpr uint8_t ledPin = 20; // NeoPixel ring
// static constexpr uint8_t builtInLED = 25;

// // --- NeoPixel setup ---
// #define LED_COUNT 24
// #define BRIGHTNESS 255
// Adafruit_NeoPixel strip(LED_COUNT, ledPin, NEO_GRBW + NEO_KHZ800);

// // --- Variables ---
// uint8_t lightMode = 0;

// void setup()
// {
//     Serial.begin(115200);
//     parser.begin(Serial2); // Sets pins for Serial2 so other libs don't need to

//     // Initialize PCA9685 for servos
//     pwm.begin();
//     pwm.setPWMFreq(50); // 50 Hz for servos

//     // Initialize encoder and stepper
//     encoder1.begin(0, 1); // SDA = GPIO 0, SCL = GPIO 1
//     joint1.begin(stepPin1, dirPin1, &encoder1);
//     joint1.calibrateFromEncoder();
//     joint1.setPIDGains(5, 1, 0.5);

//     // Attach servos
//     servo1.attach(0, &pwm);
//     servo2.attach(1, &pwm);
//     servo3.attach(2, &pwm);

//     // Setup NeoPixel strip
//     strip.begin();
//     strip.setBrightness(BRIGHTNESS);
//     strip.show(); // Start with all pixels off

//     // Setup laser pin
//     pinMode(laserPin, OUTPUT);
//     digitalWrite(laserPin, LOW);

//     // Built-in LED always ON
//     pinMode(builtInLED, OUTPUT);
//     digitalWrite(builtInLED, HIGH);
// }

// void loop()
// {
//     parser.readCommand(desired);
//     joint1.update();
//     servo1.update();
//     servo2.update();
//     servo3.update();

//     if (parser.messageReceived())
//     {
//         joint1.setTarget(desired.a1);
//         servo1.setTargetAngle(desired.a2);
//         servo2.setTargetAngle(desired.a3);
//         servo3.setTargetAngle(desired.a4);

//         current.lightMode = desired.lightMode;
//         current.a1 = joint1.getCurrentAngle();
//         current.a2 = servo1.getCurrentAngle();
//         current.a3 = servo2.getCurrentAngle();
//         current.a4 = servo3.getCurrentAngle();

//         parser.sendStatus(current);
//         parser.clearReceivedFlag();

//         lightMode = desired.lightMode;
//     }

//     // Handle lights and laser modes
//     switch (lightMode)
//     {
//         case 1: // All OFF
//             digitalWrite(laserPin, LOW);
//             strip.clear();
//             strip.show();
//             break;

//         case 2: // Laser ON only
//             digitalWrite(laserPin, HIGH);
//             strip.clear();
//             strip.show();
//             break;

//         case 3: // White LEDs ON only
//             digitalWrite(laserPin, LOW);
//             strip.fill(strip.Color(0, 0, 0, 255)); // Pure white
//             strip.show();
//             break;

//         case 4: // Laser + White LEDs
//             digitalWrite(laserPin, HIGH);
//             strip.fill(strip.Color(0, 0, 0, 255));
//             strip.show();
//             break;

//         case 5: // Laser blinking + NeoPixel light show
//             static uint32_t lastToggle = 0;
//             if (millis() - lastToggle > 500)
//             {
//                 digitalWrite(laserPin, !digitalRead(laserPin)); // Toggle laser
//                 lastToggle = millis();
//             }
//             rainbowCycle(5);
//             break;

//         default:
//             break;
//     }
// }

// // --- Rainbow animation for case 5 ---
// void rainbowCycle(uint8_t wait)
// {
//     static uint16_t j = 0;
//     j++;

//     for (uint16_t i = 0; i < strip.numPixels(); i++)
//     {
//         strip.setPixelColor(i, strip.gamma32(strip.ColorHSV((i * 65536L / strip.numPixels() + j*10) & 0xFFFF)));
//     }
//     strip.show();
//     delay(wait);
// }
