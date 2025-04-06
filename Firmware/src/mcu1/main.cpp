#include "ControllerLib.h"
#include "EncoderLib.h"
#include <Arduino.h>

ControllerLib controller;

double currentAngle1, currentAngle2, currentAngle3, currentAngle4, speed1;
double desiredAngle1, desiredAngle2, desiredAngle3, desiredAngle4;
int lightMode;

EncoderLib encoder1(Wire);
EncoderLib encoder2(Wire1);

void setup()
{
    controller.begin();
    encoder1.begin(0, 1); // GPIO 0 = SDA, GPIO 1 = SCL
    encoder2.begin(2, 3); // GPIO 2 = SDA, GPIO 3 = SCL

}

void loop()
{


}
