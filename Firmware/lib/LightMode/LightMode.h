#ifndef ENCODERLIB_H
#define ENCODERLIB_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define ledPin 20 // NeoPixel ring
#define laserPin 21 // Laser Pointer

#define LED_COUNT 24
#define BRIGHTNESS 255

Adafruit_NeoPixel lamp(LED_COUNT, ledPin, NEO_GRB + NEO_KHZ800);

class LightMode {
    public:
        LightMode();
        
        void setLightMode(int mode);
    


    private:
        uint8_t wait;

        void _rainbowCycle(uint8_t wait);

    };
#endif