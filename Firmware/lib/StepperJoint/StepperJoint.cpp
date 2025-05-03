// StepperJoint.cpp

#include "StepperJoint.h"

StepperJoint::StepperJoint()
  : _stepPin(255), _dirPin(255),
    _ms1Pin(255), _ms2Pin(255),
    _encoder(nullptr),
    _targetAngle(0), _currentAngle(0), _angleOffset(0),
    _kp(0), _ki(0), _kd(0),
    _lastError(0), _integral(0),
    _microsteps(1),
    _lastPIDTime(0), _lastStepTime(0),
    _stepIntervalMicros(2000),
    _stepState(false),
    _deadbandDeg(0)    // default no deadband
{}

void StepperJoint::begin(uint8_t stepPin, uint8_t dirPin, EncoderLib* encoder, uint8_t ms1Pin, uint8_t ms2Pin) {
    _stepPin = stepPin; _dirPin = dirPin;
    _encoder = encoder;
    _ms1Pin  = ms1Pin;  _ms2Pin  = ms2Pin;
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin,  OUTPUT);
    pinMode(_ms1Pin,  OUTPUT);
    pinMode(_ms2Pin,  OUTPUT);
    setMicrostepping(1);
    _lastPIDTime  = millis();
    _lastStepTime = micros();
}

void StepperJoint::calibrateFromEncoder() {
    _angleOffset = fmod(_encoder->getFilteredAngle(), 360.0f);
}

void StepperJoint::setTarget(float angle) {
    _targetAngle = fmod(angle + 360.0f, 360.0f);
}

float StepperJoint::getCurrentAngle() const {
    return _currentAngle;
}

void StepperJoint::setPIDGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void StepperJoint::setMicrostepping(int mode) {
    _microsteps = mode;
    switch (mode) {
      case 1:  digitalWrite(_ms1Pin, LOW);  digitalWrite(_ms2Pin, LOW);  break;
      case 2:  digitalWrite(_ms1Pin, HIGH); digitalWrite(_ms2Pin, LOW);  break;
      case 4:  digitalWrite(_ms1Pin, LOW);  digitalWrite(_ms2Pin, HIGH); break;
      case 8:  digitalWrite(_ms1Pin, HIGH); digitalWrite(_ms2Pin, HIGH); break;
      default: digitalWrite(_ms1Pin, LOW);  digitalWrite(_ms2Pin, LOW);  
               _microsteps = 1;
    }
}

void StepperJoint::update() {
    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f;
    if (dt <= 0) return;
    _lastPIDTime = now;

    // read current angle
    float rawAngle     = _encoder->getFilteredAngle();
    float logicalAngle = fmod((rawAngle - _angleOffset + 360.0f), 360.0f);
    _currentAngle      = logicalAngle;

    // compute error
    float error = angleDiff(_targetAngle, _currentAngle);

    // ── Deadband check ─────────────────────────────────
    if (fabs(error) < _deadbandDeg) {
        // within deadband → no stepping
        _integral  = 0;
        _lastError = error;
        return;
    }

    // PID calculations
    _integral    += error * dt;
    float derivative = (error - _lastError) / dt;
    _lastError   = error;

    float velocity = (_kp * error + _ki * _integral + _kd * derivative) * 120.0f;

    // convert velocity → step interval
    static constexpr float stepsPerRev = 200.0f;
    float microstepsPerDeg = (stepsPerRev * _microsteps) / 360.0f;
    float stepsPerSecond = constrain(fabs(velocity) * microstepsPerDeg, 50.0f, 3000.0f);
    _stepIntervalMicros = 1e6f / stepsPerSecond;

    digitalWrite(_dirPin, velocity >= 0 ? LOW : HIGH);

    unsigned long nowMicros = micros();
    if (nowMicros - _lastStepTime >= _stepIntervalMicros) {
        _lastStepTime = nowMicros;
        _stepState    = !_stepState;
        digitalWrite(_stepPin, _stepState);
    }
}

float StepperJoint::angleDiff(float target, float current) {
    float diff = target - current;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// PID getters/setters
void  StepperJoint::setKp(float kp)   { _kp = kp; }
void  StepperJoint::setKi(float ki)   { _ki = ki; }
void  StepperJoint::setKd(float kd)   { _kd = kd; }
float StepperJoint::getKp() const     { return _kp; }
float StepperJoint::getKi() const     { return _ki; }
float StepperJoint::getKd() const     { return _kd; }

// ── New deadband setter ──────────────────────────────
void StepperJoint::setDeadband(float degrees) {
    _deadbandDeg = degrees;
}
