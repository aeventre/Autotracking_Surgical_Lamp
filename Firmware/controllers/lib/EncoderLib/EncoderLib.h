#ifndef ENCODERLIB_H
#define ENCODERLIB_H

#include <Arduino.h>
#include <Wire.h>

class EncoderLib {
public:
  explicit EncoderLib(TwoWire& wirePort);

  void begin(uint8_t sdaPin, uint8_t sclPin);
  void calibrateWithStepper();

  float getRawAngle() const;
  float getFilteredAngle();
  void zero();  // <-- Added zeroing method

private:
  void readRawAngle();
  void interpolate();
  void stepMotor(int motor, bool dir);
  void checkMagnetPresence();

  TwoWire& wire;

  static constexpr int sampleInterval = 9;
  static constexpr int samples = int(360 / sampleInterval + 2);
  static constexpr byte DIR1 = 14;
  static constexpr byte STEP1 = 8;
  static constexpr unsigned int PULSE_WIDTH = 2;
  static constexpr unsigned long DELAY_MIN = 2500;

  float calibrationTable[samples];
  float interpolatedAngle = 0.0f;
  float degAngle = 0.0f;
  float zeroOffset = 0.0f;  // <-- Added offset storage
  int rawAngle = 0;
  int magnetStatus = 0;
  int StepCounter = 0;
};

#endif
