#include <Arduino.h>
#include "EncoderLib.h"

EncoderLib encoder;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Teensy or Leonardo-safe

  Serial.println("Initializing encoder...");
  encoder.begin();  // Starts I2C and checks for magnet
  Serial.println("Initialization complete.");
}

void loop() {
  float raw = encoder.getRawAngle();
  float filtered = encoder.getFilteredAngle();

  Serial.print("Raw Angle: ");
  Serial.print(raw, 2);
  Serial.print(" deg\tFiltered Angle: ");
  Serial.print(filtered, 2);
  Serial.println(" deg");

  delay(200);
}