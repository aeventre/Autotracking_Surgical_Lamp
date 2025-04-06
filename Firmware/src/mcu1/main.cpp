#include <Arduino.h>
#include "DRV8834.h"

#define dirPin 16
#define stepPin 17
#define neoPixelPin 18
#define laserPin 19
#define joint3FeedbackPin 26
#define joint4FeedbackPin 27


void setup() {
  // Initialize serial to RS-485 comms
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);

}

void loop() {


}
