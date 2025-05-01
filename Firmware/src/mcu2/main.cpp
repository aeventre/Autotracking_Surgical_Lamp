#include "EncoderLib.h"
#include "StepperJoint.h"
#include <Arduino.h>

float readJoint0Command();

#define stepPin 3
#define dirPin 2

StepperJoint joint0;
EncoderLib encoder(Wire);

unsigned long lastPrint = 0;

void setup()
{
    pinMode(2,OUTPUT);
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    //digitalWrite(4,HIGH);
    Serial.begin(115200);
    encoder.begin(A4, A5); // Uno SDA/SCL pins
    joint0.begin(stepPin, dirPin, &encoder, 4,5);
    joint0.calibrateFromEncoder(); // Set encoder angle to 0
    joint0.setPIDGains(5, 1, 0.5);
    joint0.setTarget(0); 
}

void loop()
{


}

float readJoint0Command()
{
    static String input = "";
    static bool receiving = false;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '<')
        {
            input = "";
            receiving = true;
        }
        else if (c == '>' && receiving)
        {
            receiving = false;
            return input.toFloat(); // Parse angle
        }
        else if (receiving)
        {
            input += c;
        }
    }

    return NAN; // Return invalid if no message complete
}
