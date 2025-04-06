#include "ControllerLib.h"
#include "EncoderLib.h"

ControllerLib::ControllerLib() :
  encoder1(Wire), // Pass Wire (or Wire1) to EncoderLib constructor
  encoder2(Wire1)
{}

void ControllerLib::begin()
{
    // Initialize serial to RS-485 comms
    Serial2.setTX(4);
    Serial2.setRX(5);
    Serial2.begin(115200);

    pinMode(joint3FeedbackPin, INPUT);
    pinMode(joint4FeedbackPin, INPUT);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    encoder1.begin(0, 1); // GPIO 0 = SDA, GPIO 1 = SCL
    encoder2.begin(2, 3); // GPIO 2 = SDA, GPIO 3 = SCL
}

void ControllerLib::receiveCommand(float *angle1, float *angle2, float *angle3, float *angle4, int *lightMode)
{
    static String input = "";
    static bool receiving = false;

    while (Serial2.available())
    {
        char c = Serial2.read();

        if (c == '<')
        {
            input = "";
            receiving = true;
        }
        else if (c == '>' && receiving)
        {
            receiving = false;

            // Parse input
            int c1 = input.indexOf(',');
            int c2 = input.indexOf(',', c1 + 1);
            int c3 = input.indexOf(',', c2 + 1);
            int c4 = input.indexOf(',', c3 + 1);

            if (c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1)
                return;

            *angle1 = input.substring(0, c1).toFloat();
            *angle2 = input.substring(c1 + 1, c2).toFloat();
            *angle3 = input.substring(c2 + 1, c3).toFloat();
            *angle4 = input.substring(c3 + 1, c4).toFloat();
            *lightMode = input.substring(c4 + 1).toInt();

            receiveState = true; // Mark that we received a complete command
        }
        else if (receiving)
        {
            input += c;
        }
    }
}

void ControllerLib::getServoAngle(float *angle3, float *angle4)
{
    // Read Analog values and convert to angles
    *angle3 = (float)analogRead(joint3FeedbackPin) * 270 / 4095;
    *angle4 = (float)analogRead(joint4FeedbackPin) * 270 / 4095;
}

void ControllerLib::updateJoint1PID()
{
    unsigned long now = millis();
    float dt = (now - lastPIDTime1) / 1000.0;
    if (dt <= 0) return;
    lastPIDTime1 = now;

    currentAngle1 = encoder1.getFilteredAngle();
    float error = targetAngle1 - currentAngle1;
    integral1 += error * dt;
    float derivative = (error - lastError1) / dt;
    lastError1 = error;

    float outputVelocity = kp1 * error + ki1 * integral1 + kd1 * derivative;

    // Convert to steps/sec
    float microstepsPerDeg = (200.0 * 16) / 360.0;
    float stepsPerSecond = abs(outputVelocity) * microstepsPerDeg;

    if (stepsPerSecond > 1) {
        stepIntervalMicros1 = 1e6 / stepsPerSecond;
        digitalWrite(dirPin, outputVelocity >= 0 ? HIGH : LOW);
    } else {
        stepIntervalMicros1 = 1e9;  // Stop stepping
    }
}

void ControllerLib::updateJoint1Stepper()
{
    if (stepIntervalMicros1 < 1e6) {
        unsigned long now = micros();
        if (now - lastStepTime1 >= stepIntervalMicros1) {
            lastStepTime1 = now;
            stepState1 = !stepState1;
            digitalWrite(stepPin, stepState1);
        }
    }
}
