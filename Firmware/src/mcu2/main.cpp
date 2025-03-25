#include <Arduino.h>
#include <Servo.h>

Servo joint2;
Servo joint3;
Servo joint4;

void recieveCommand(float &angle2, float &angle3, float &angle4, int &lightMode);
void controlLight(int lightMode);

#define joint3Feedback A0
#define joint4Feedback A1

float commandAngle2, commandAngle3, commandAngle4;
int lightMode;

void setup()
{
    joint2.attach(3);
    joint3.attach(5);
    joint4.attach(6);

    Serial.begin(115200);
}

void loop()
{
    float currentAngle3 = map(analogRead(joint3Feedback), 0, 1023, 0, 270);
    float currentAngle4 = map(analogRead(joint4Feedback), 0, 1023, 0, 270);
    recieveCommand(commandAngle2, commandAngle3, commandAngle4, lightMode);

    // Send Current Positions to ROS
    Serial.print("<");
    Serial.print(currentAngle3);
    Serial.print(",");
    Serial.print(currentAngle4);
    Serial.println(">");
}

void recieveCommand(float &angle2, float &angle3, float &angle4, int &lightMode)
{
    static String input = "";
    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '<')
        {
            input = ""; // Start new packet
        }
        else if (c == '>')
        {
            // End of packet: parse
            int c1 = input.indexOf(',');
            int c2 = input.indexOf(',', c1 + 1);
            int c3 = input.indexOf(',', c2 + 1);

            if (c1 == -1 || c2 == -1 || c3 == -1)
                return;

            angle2 = input.substring(0, c1).toFloat();
            angle3 = input.substring(c1 + 1, c2).toFloat();
            angle4 = input.substring(c2 + 1, c3).toFloat();
            lightMode = input.substring(c3 + 1).toInt();
        }
        else
        {
            input += c;
        }
    }
}

void controlLight(int lightMode)
{
    switch (lightMode)
    {
    case 0:

        break;

    default:
        break;
    }
}
