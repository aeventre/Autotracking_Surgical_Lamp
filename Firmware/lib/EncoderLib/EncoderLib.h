#ifndef ENCODERLIB_H
#define ENCODERLIB_H

#include <Arduino.h>
#include <Wire.h>

class EncoderLib {
public:
  explicit EncoderLib(TwoWire& wirePort);
  // Constructor that accepts a reference to a TwoWire instance (e.g., Wire, Wire1)

  void begin(uint8_t sdaPin, uint8_t sclPin);
  // Initializes the I2C bus and checks for magnet presence

  void calibrateWithStepper();
  // Steps the motor and builds a calibration table by sampling angles

  float getRawAngle() const;
  // Returns the most recent raw angle from the sensor (0–360°)

  float getFilteredAngle();
  // Reads from the sensor, runs interpolation, and returns the corrected angle

private:
  void readRawAngle();               // Reads the 12-bit raw angle from AS5600
  void interpolate();                // Performs angle interpolation from the calibration table
  void stepMotor(int motor, bool dir); // Steps the motor for calibration
  void checkMagnetPresence();        // Blocks until the magnet is detected or times out

  TwoWire& wire;                     // I2C bus reference (Wire, Wire1, etc.)

  // Constants
  static constexpr int sampleInterval = 9;
  static constexpr int samples = int(360 / sampleInterval + 2);
  static constexpr byte DIR1 = 14;  // Motor direction pin
  static constexpr byte STEP1 = 8;  // Motor step pin
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
