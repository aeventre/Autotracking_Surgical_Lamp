#include <Arduino.h>
#include <unity.h>
#include "encoderLib.h"

AS5600Interpolator encoder;

void test_angle_is_zero_after_begin() {
    encoder.begin();  // Simulate calibration
    encoder.update(); // Read sensor once
    float angle = encoder.getInterpolatedAngle();
    TEST_ASSERT_FLOAT_WITHIN(1.0, 0.0, angle); // Within 1 degree of 0
}

void setup() {
    delay(2000); // Wait for serial monitor to open
    UNITY_BEGIN();

    RUN_TEST(test_angle_is_zero_after_begin);

    UNITY_END();
}

void loop() {
    // Leave empty — Unity runs everything in setup()
}
