#include "LightMode.h"

LightMode::LightMode() {

    pinMode(laserPin, OUTPUT);
    digitalWrite(laserPin, LOW);

    lamp.begin();
    lamp.setBrightness(BRIGHTNESS);
    lamp.show(); // Start with all pixels off

}

void setLightMode(int mode) {

    switch (mode)
    {
        case 1: // All OFF
            digitalWrite(laserPin, LOW);
            lamp.clear();
            lamp.show();
            break;

        case 2: // Laser ON only
            digitalWrite(laserPin, HIGH);
            lamp.clear();
            lamp.show();
            break;

        case 3: // White LEDs ON only
            digitalWrite(laserPin, LOW);
            lamp.fill(lamp.Color(0, 0, 0, 255)); // Pure white
            lamp.show();
            break;

        case 4: // Laser + White LEDs
            digitalWrite(laserPin, HIGH);
            lamp.fill(lamp.Color(0, 0, 0, 255));
            lamp.show();
            break;

        case 5: // Laser blinking + NeoPixel light show
            static uint32_t lastToggle = 0;
            if (millis() - lastToggle > 500)
            {
                digitalWrite(laserPin, !digitalRead(laserPin)); // Toggle laser
                lastToggle = millis();
            }
            _rainbowCycle(5);
            break;

        default:
            break;
    }

}

void _rainbowCycle(uint8_t wait) {
    static uint16_t j = 0;
    j++;

    for (uint16_t i = 0; i < lamp.numPixels(); i++)
    {
        lamp.setPixelColor(i, lamp.gamma32(lamp.ColorHSV((i * 65536L / lamp.numPixels() + j*10) & 0xFFFF)));
    }
    lamp.show();
    delay(wait);
}