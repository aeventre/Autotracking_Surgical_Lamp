#include <Arduino.h>
#include "AS5600.h"

AS5600L as5600;  // Use default Wire

bool encoderConnected = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to connect (especially on Leonardo)

  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin();

  as5600.begin(4);  // Set direction pin
  as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.setAddress(0x36);


  Serial.print("I2C Address: 0x");
  Serial.println(as5600.getAddress(), HEX);

  encoderConnected = as5600.isConnected();
  Serial.print("Initial Connect: ");
  Serial.println(encoderConnected ? "OK" : "FAILED");

  if (!encoderConnected) {
    Serial.println("ERROR: AS5600 not detected! Check wiring or power.");
  }

  delay(1000);
}

void loop()
{
  static uint32_t lastTime = 0;

  // Check connection periodically (e.g., every second)
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();

    encoderConnected = as5600.isConnected();
    if (!encoderConnected) {
      Serial.println("WARNING: AS5600 DISCONNECTED or not responding.");
      return; // Skip the rest of the loop until it's back
    }
  }

  // Regular reading every 100 ms
  if (encoderConnected && millis() - lastTime >= 100)
  {
    lastTime = millis();

    // If we can read, show values
    long pos = as5600.getCumulativePosition();
    int revs = as5600.getRevolutions();

    Serial.print("Position: ");
    Serial.print(pos);
    Serial.print("\tRevs: ");
    Serial.println(revs);

    // Optional: reset after 10 turns
    if (revs >= 10) {
      Serial.println("Resetting position...");
      as5600.resetPosition();
    }
  }
}
