#include "encoderLib.h"

#define SET(x,y) (x |= (1<<y))
#define CLR(x,y) (x &= (~(1<<y)))

AS5600Interpolator::AS5600Interpolator() {}

void AS5600Interpolator::begin() {
    Wire.begin();
    Wire.setClock(400000L);
    initMotor();
    checkMagnetPresence();
    readRawAngle();
    startAngle = degAngle;

    for (int i = 0; i < samples; i++) {
        readRawAngle();
        tareAngle();
        checkQuadrant();
        calibrationTable[i] = totalAngle;

        int motorSteps = int(sampleInterval * 16 / 1.8);
        for (int step = 0; step < motorSteps; step++) {
            stepMotor(1, false); // CCW
            stepCounter++;
        }
    }

    numberSteps = stepCounter;
    for (unsigned long i = 0; i < numberSteps; i++) {
        stepMotor(1, true); // CW
        stepCounter--;
    }

    readRawAngle();
    tareAngle();
}

void AS5600Interpolator::update() {
    readRawAngle();
    tareAngle();
    checkQuadrant();
    interpolate();
}

float AS5600Interpolator::getInterpolatedAngle() {
    return interpolatedAngle;
}

float AS5600Interpolator::getRawAngle() {
    return degAngle;
}

float AS5600Interpolator::getTaredAngle() {
    return taredAngle;
}

float AS5600Interpolator::getTotalAngle() {
    return totalAngle;
}

void AS5600Interpolator::initMotor() {
    pinMode(DIR1, OUTPUT);
    pinMode(STEP1, OUTPUT);
    digitalWrite(DIR1, true);
    delayMicroseconds(PULSE_WIDTH);
    digitalWrite(STEP1, LOW);
}

void AS5600Interpolator::stepMotor(int motor, bool dir) {
    byte patternC = PORTC;
    byte patternB = PORTB;

    const int dir1 = 0;
    const int step1 = 0;

    if (motor == 1) {
        CLR(patternB, step1);
        PORTB = patternB;

        if (dir) SET(patternC, dir1); else CLR(patternC, dir1);
        PORTC = patternC;
        delayMicroseconds(PULSE_WIDTH);

        SET(patternB, step1);
        PORTB = patternB;
        delayMicroseconds(PULSE_WIDTH);

        CLR(patternB, step1);
        PORTB = patternB;
        delayMicroseconds(DELAY_MIN);
    }
}

void AS5600Interpolator::readRawAngle() {
    Wire.beginTransmission(0x36);
    Wire.write(0x0D);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (!Wire.available());
    lowbyte = Wire.read();

    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (!Wire.available());
    highbyte = Wire.read();

    rawAngle = ((word)highbyte << 8) | lowbyte;
    degAngle = rawAngle * 0.087890625;
}

void AS5600Interpolator::tareAngle() {
    taredAngle = degAngle - startAngle;
    if (taredAngle < 0)
        taredAngle += 360;
}

void AS5600Interpolator::checkQuadrant() {
    if (taredAngle <= 90) quadrantNumber = 1;
    else if (taredAngle <= 180) quadrantNumber = 2;
    else if (taredAngle <= 270) quadrantNumber = 3;
    else quadrantNumber = 4;

    if (quadrantNumber != previousQuadrantNumber) {
        if (quadrantNumber == 1 && previousQuadrantNumber == 4) numberOfTurns++;
        if (quadrantNumber == 4 && previousQuadrantNumber == 1) numberOfTurns--;
        previousQuadrantNumber = quadrantNumber;
    }

    totalAngle = numberOfTurns * 360 + taredAngle;
}

void AS5600Interpolator::interpolate() {
    for (int index = 1; index < samples; index++) {
        if (calibrationTable[index] > taredAngle) {
            float diff1 = taredAngle - calibrationTable[index - 1];
            float diff2 = calibrationTable[index] - calibrationTable[index - 1];
            interpolatedAngle = float(sampleInterval) * ((index - 1) + diff1 / diff2);
            return;
        }
    }
}

void AS5600Interpolator::checkMagnetPresence() {
    while ((magnetStatus & B00100000) != 32) {
        magnetStatus = 0;
        Wire.beginTransmission(0x36);
        Wire.write(0x0B);
        Wire.endTransmission();
        Wire.requestFrom(0x36, 1);
        while (!Wire.available());
        magnetStatus = Wire.read();
        delay(100);
    }
}