// encoderLib.h
#ifndef ENCODERLIB_H
#define ENCODERLIB_H

#include <Arduino.h>

class EncoderLib {
public:
  EncoderLib();  
  // Constructor: Initializes the EncoderLib object

  void begin();  
  // Initializes I2C, checks for magnet, and performs calibration by stepping the motor

  float getRawAngle() const;  
  // Returns the most recent raw angle in degrees (0–360)

  float getFilteredAngle();  
  // Reads from the sensor and returns the interpolated (corrected) angle

private:
  void readRawAngle(); 
  void interpolate();           
  void stepMotor(int motor, bool dir);
  void checkMagnetPresence();

  // Constants
  static constexpr int sampleInterval = 9;
  static constexpr int samples = int(360 / sampleInterval + 2);

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
