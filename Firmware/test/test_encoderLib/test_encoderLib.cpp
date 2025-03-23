// #include <Arduino.h>
// #include "encoderLib.h"  // or "AS5600Interpolator.h" if that's your header

// AS5600Interpolator encoder;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);       // Give serial monitor time to connect
//   encoder.begin();   // Run calibration
// }

// void loop() {
//   encoder.update();                        // Refresh reading
//   float angle = encoder.getInterpolatedAngle();  // Get interpolated result

//   Serial.print("Angle: ");
//   Serial.println(angle, 2);                // Print with 2 decimal places

//   delay(100);                              // ~10 Hz update rate
// }
