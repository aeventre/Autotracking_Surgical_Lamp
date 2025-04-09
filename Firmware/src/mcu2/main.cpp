#include "EncoderLib.h"
#include "StepperJoint.h"
#include <Arduino.h>

float readJoint0Command();

#define stepPin 2
#define dirPin 3

StepperJoint joint0;
EncoderLib encoder(Wire);

unsigned long lastPrint = 0;

void setup()
{
    Serial.begin(115200);
    encoder.begin(A4, A5);  // Uno SDA/SCL pins
    joint0.begin(stepPin, dirPin, &encoder);
    joint0.calibrateFromEncoder(); // Set encoder angle to 0
    joint0.setPIDGains(5, 1, 0.5);
}

void loop()
{
    float angle = readJoint0Command();

    if (!isnan(angle)) {
        joint0.setTarget(angle);
    }

    joint0.update();

    // Throttle printing to once every 100ms
    if (millis() - lastPrint > 100) {
        Serial.print("<");
        Serial.print(joint0.getCurrentAngle(), 2);
        Serial.println(">");
        lastPrint = millis();
    }
}

float readJoint0Command()
{
    static String input = "";
    static bool receiving = false;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '<') {
            input = "";
            receiving = true;
        }
        else if (c == '>' && receiving) {
            receiving = false;
            return input.toFloat();  // Parse angle
        }
        else if (receiving) {
            input += c;
        }
    }

    return NAN;  // Return invalid if no message complete
}
