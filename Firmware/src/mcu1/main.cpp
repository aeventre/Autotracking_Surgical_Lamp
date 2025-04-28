#include "CommandParser.h"
#include "EncoderLib.h"
#include "StepperJoint.h"
#include "ServoJoint.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

// --- Objects ---
StepperJoint joint1;
EncoderLib encoder1(Wire);
ServoJoint servo1, servo2, servo3;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
CommandParser parser;
CommandMessage desired;
CommandMessage current;

void handleLightMode(int mode);
void laserOn();
void laserOff();
void laserBlink();
void pixelsOff();
void pixelsWhite();
void pixelLightShow();
void rainbowCycle(uint8_t wait);

// --- Constants ---
static constexpr uint8_t dirPin1 = 16;
static constexpr uint8_t stepPin1 = 17;
static constexpr uint8_t laserPin = 21;
static constexpr uint8_t ledPin = 20; // NeoPixel ring
static constexpr uint8_t builtInLED = 25;

// --- NeoPixel setup ---
#define LED_COUNT 24
#define BRIGHTNESS 255
Adafruit_NeoPixel strip(LED_COUNT, ledPin, NEO_GRBW + NEO_KHZ800);

// --- Variables ---
uint8_t lightMode = 0;

void setup()
{   
    Serial.begin(115200);
    parser.begin(Serial2);

    // Initialize PCA9685 for servos
    pwm.begin();
    pwm.setPWMFreq(50); // 50 Hz for servos

    // Initialize encoder and stepper
    encoder1.begin(0, 1); // SDA = GPIO 0, SCL = GPIO 1
    joint1.begin(stepPin1, dirPin1, &encoder1);
    joint1.calibrateFromEncoder();
    joint1.setPIDGains(5, 1, 0.5);

    // Attach servos
    servo1.attach(0, &pwm); 
    servo2.attach(1, &pwm); 
    servo3.attach(2, &pwm);

    // Setup NeoPixel strip
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show(); // Start with all pixels off

    // Setup laser pin
    pinMode(laserPin, OUTPUT);
    digitalWrite(laserPin, LOW);

    // Built-in LED always ON
    pinMode(builtInLED, OUTPUT);
    digitalWrite(builtInLED, HIGH);
}

void loop()
{
    parser.readCommand(desired);
    joint1.update();
    servo1.update();
    servo2.update();
    servo3.update();

    if (parser.messageReceived())
    {
        joint1.setTarget(desired.a1);
        servo1.setTargetAngle(desired.a2);
        servo2.setTargetAngle(desired.a3);
        servo3.setTargetAngle(desired.a4);

        current.lightMode = desired.lightMode;
        current.a1 = joint1.getCurrentAngle();
        current.a2 = servo1.getCurrentAngle();
        current.a3 = servo2.getCurrentAngle();
        current.a4 = servo3.getCurrentAngle();
        
        parser.sendStatus(current);
        parser.clearReceivedFlag();
        
        lightMode = desired.lightMode;
    }

    // Handle lights and laser modes
    switch (lightMode)
    {
        case 1: // All OFF
            digitalWrite(laserPin, LOW);
            strip.clear();
            strip.show();
            break;

        case 2: // Laser ON only
            digitalWrite(laserPin, HIGH);
            strip.clear();
            strip.show();
            break;

        case 3: // White LEDs ON only
            digitalWrite(laserPin, LOW);
            strip.fill(strip.Color(0, 0, 0, 255)); // Pure white
            strip.show();
            break;

        case 4: // Laser + White LEDs
            digitalWrite(laserPin, HIGH);
            strip.fill(strip.Color(0, 0, 0, 255));
            strip.show();
            break;

        case 5: // Laser blinking + NeoPixel light show
            static uint32_t lastToggle = 0;
            if (millis() - lastToggle > 500)
            {
                digitalWrite(laserPin, !digitalRead(laserPin)); // Toggle laser
                lastToggle = millis();
            }
            rainbowCycle(5);
            break;

        default:
            break;
    }
}

// --- Rainbow animation for case 5 ---
void rainbowCycle(uint8_t wait)
{
    static uint16_t j = 0;
    j++;

    for (uint16_t i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV((i * 65536L / strip.numPixels() + j*10) & 0xFFFF)));
    }
    strip.show();
    delay(wait);
}
