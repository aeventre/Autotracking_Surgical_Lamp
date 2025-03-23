// EncoderLib.cpp
#include "EncoderLib.h"
#include <Wire.h>

EncoderLib::EncoderLib() {}

void EncoderLib::begin() {
  Wire.begin();
  checkMagnetPresence();

  // Read initial angle to ensure degAngle is populated
  readRawAngle();

  // Build calibration table by rotating and recording degAngle every sampleInterval
  for (int i = 0; i < samples; i++) {
    readRawAngle();
    calibrationTable[i] = degAngle;

    // Rotate sampleInterval degrees (e.g. 80 microsteps for 9 degrees)
    int motorSteps = int(sampleInterval * 16 / 1.8);
    for (int step = 0; step < motorSteps; step++) {
      stepMotor(1, false); // CCW
    }
  }

  // Return to home position
  for (int i = 0; i < StepCounter; i++) {
    stepMotor(1, true); // CW
  }
  StepCounter = 0;
}

float EncoderLib::getRawAngle() const {
  return degAngle;
}

float EncoderLib::getFilteredAngle() {
  readRawAngle();
  interpolate();
  return interpolatedAngle;
}

void EncoderLib::interpolate() {
  for (int index = 1; index < samples; index++) {
    if (calibrationTable[index] > degAngle) {
      float diff1 = degAngle - calibrationTable[index - 1];
      float diff2 = calibrationTable[index] - calibrationTable[index - 1];
      interpolatedAngle = sampleInterval * ((index - 1) + diff1 / diff2);
      return;
    }
  }
  interpolatedAngle = degAngle; // fallback if not interpolated
}

void EncoderLib::readRawAngle() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (!Wire.available());
  int low = Wire.read();

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (!Wire.available());
  int high = Wire.read();

  rawAngle = ((high << 8) | low) & 0x0FFF;
  degAngle = rawAngle * 0.087890625f; // 360 / 4096
}

void EncoderLib::checkMagnetPresence() {
  unsigned long start = millis();
  while ((magnetStatus & 0x20) != 0x20) {
    magnetStatus = 0;
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    if (Wire.available()) {
      magnetStatus = Wire.read();
    }
    if (millis() - start > 3000) {
      break; // timeout if magnet not detected
    }
    delay(100);
  }
}

void EncoderLib::stepMotor(int motor, bool dir) {
  digitalWrite(DIR1, dir ? HIGH : LOW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, LOW);
  delayMicroseconds(DELAY_MIN);
  StepCounter += dir ? -1 : 1;
}
