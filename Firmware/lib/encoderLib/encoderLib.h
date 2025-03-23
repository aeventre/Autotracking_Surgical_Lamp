#ifndef surgBotLib
#define surgBotLib H

#include <Arduino.h>
#include <Wire.h>

class AS5600Interpolator {
    public:
      AS5600Interpolator();
    
      void begin();
      void update();
    
      float getInterpolatedAngle() const;
      float getRawAngle() const;
    
    private:
      void readRawAngle();
      void interpolate();
      void stepMotor(int motor, bool dir);
      void checkMagnetPresence();
    
      // Constants
      static constexpr int sampleInterval = 9;
      static constexpr int samples = int(360 / sampleInterval + 2);
    
      // Motor control pins and parameters
      static constexpr byte DIR1 = 14;  // A0
      static constexpr byte STEP1 = 8;  // D8
      static constexpr unsigned int PULSE_WIDTH = 2;
      static constexpr unsigned long DELAY_MIN = 2500;
    
      // Internal state
      float calibrationTable[samples];
      float interpolatedAngle = 0.0f;
      float degAngle = 0.0f;
      int rawAngle = 0;
      int magnetStatus = 0;
    
      int StepCounter = 0;
    };
    
#endif