#ifndef surgBotLib
#define surgBotLib H

#include <Arduino.h>
#include <Wire.h>

class AS5600Interpolator {
    public:
        AS5600Interpolator();
    
        void begin();                     // Call this in setup()
        void calibrate();                 // Optional: re-calibrate manually
        void update();                    // Call periodically to update sensor and internal state
        float getInterpolatedAngle();    // Get most recent interpolated angle (deg)
        float getRawAngle();             // Get raw encoder angle (deg)
        float getTaredAngle();           // Get tared angle (deg)
        float getTotalAngle();           // Angle including full turns
    
    private:
        void checkMagnetPresence();
        void interpolate();
        void stepMotor(int motor, bool dir);
        void readRawAngle();
        void tareAngle();
        void checkQuadrant();
        void initMotor();
    
        // Sensor state
        int magnetStatus = 0;
        int lowbyte = 0;
        word highbyte = 0;
        int rawAngle = 0;
        float degAngle = 0;
    
        // Calibration
        static const int sampleInterval = 9;
        static const int samples = int(360 / sampleInterval + 2);
        float calibrationTable[samples];
        float interpolatedAngle = 0;
    
        // Angle tracking
        float numberOfTurns = 0;
        float startAngle = 0;
        float taredAngle = 0;
        float totalAngle = 0;
        float previousTotalAngle = 0;
        int quadrantNumber = 0;
        int previousQuadrantNumber = 0;
    
        // Motor state
        const byte DIR1 = 14;     // A0
        const byte STEP1 = 8;     // D8
        const unsigned int PULSE_WIDTH = 2;
        const unsigned long DELAY_MIN = 2500L;
    
        unsigned long stepCounter = 0;
        unsigned long numberSteps = 0;
    };
    

#endif