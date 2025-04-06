// EncoderLib.cpp
#include "EncoderLib.h"
#include <Wire.h>

EncoderLib::EncoderLib(TwoWire& wirePort) : wire(wirePort) {}

void EncoderLib::begin(uint8_t sdaPin, uint8_t sclPin) {
  wire.setSDA(sdaPin);
  wire.setSCL(sclPin);
  wire.begin();

  checkMagnetPresence();
  readRawAngle();
}

void EncoderLib::calibrateWithStepper() {
  for (int i = 0; i < samples; i++) {
    readRawAngle();
    calibrationTable[i] = degAngle;

    int motorSteps = int(sampleInterval * 16 / 1.8);
    for (int step = 0; step < motorSteps; step++) {
      stepMotor(1, false); // CCW
    }
  }

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
  wire.beginTransmission(0x36);
  wire.write(0x0D);
  wire.endTransmission();
  wire.requestFrom(0x36, 1);
  while (!wire.available());
  int low = wire.read();

  wire.beginTransmission(0x36);
  wire.write(0x0C);
  wire.endTransmission();
  wire.requestFrom(0x36, 1);
  while (!wire.available());
  int high = wire.read();

  rawAngle = ((high << 8) | low) & 0x0FFF;
  degAngle = rawAngle * 0.087890625f; // 360 / 4096
}

void EncoderLib::checkMagnetPresence() {
  unsigned long start = millis();
  while ((magnetStatus & 0x20) != 0x20) {
    magnetStatus = 0;
    wire.beginTransmission(0x36);
    wire.write(0x0B);
    wire.endTransmission();
    wire.requestFrom(0x36, 1);
    if (wire.available()) {
      magnetStatus = wire.read();
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
