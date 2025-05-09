// main.ino

#include "EncoderLib.h"
#include "StepperJoint.h"
#include <Arduino.h>

float readJoint0Command();
void readPIDCommand();

#define stepPin 3
#define dirPin  2

StepperJoint joint0;
EncoderLib   encoder(Wire);

void setup() {
    // Motor pins
    pinMode(dirPin,  OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(4,       OUTPUT);
    pinMode(5,       OUTPUT);

    Serial.begin(115200);

    // Encoder init & zero at startup
    encoder.begin(A4, A5);
    encoder.zero();

    // Joint init & zero calibration
    joint0.begin(stepPin, dirPin, &encoder, 4, 5);
    joint0.calibrateFromEncoder();

    // Initial PID & target
    joint0.setPIDGains(5.5f, 2.0f, 0.5f);
    joint0.setDeadband(0.5f);    // <–– within ±0.5° no ticking
    joint0.setTarget(0.0f);

}

void loop() {
    // Angle‐target commands: <<angle>>
    float newTarget = readJoint0Command();
    if (!isnan(newTarget)) {
        joint0.setTarget(newTarget);
        Serial.print("New target: ");
        Serial.println(newTarget);
    }

    // PID tuning commands: <P<value>>, <I<value>>, <D<value>>
    readPIDCommand();

    // Run control loop
    joint0.update();
}

void readPIDCommand() {
    static String input;
    static bool   receiving = false;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '<') {
            input     = "";
            receiving = true;
        }
        else if (c == '>' && receiving) {
            receiving = false;
            float val = input.substring(1).toFloat();

            if (input.startsWith("P")) {
                joint0.setKp(val);
                Serial.print("Kp = ");
                Serial.println(val);
            }
            else if (input.startsWith("I")) {
                joint0.setKi(val);
                Serial.print("Ki = ");
                Serial.println(val);
            }
            else if (input.startsWith("D")) {
                joint0.setKd(val);
                Serial.print("Kd = ");
                Serial.println(val);
            }
        }
        else if (receiving) {
            input += c;
        }
    }
}

float readJoint0Command() {
    static String input;
    static bool   receiving = false;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '<') {
            input     = "";
            receiving = true;
        }
        else if (c == '>' && receiving) {
            receiving = false;
            return input.toFloat();
        }
        else if (receiving) {
            input += c;
        }
    }
    return NAN;
}
