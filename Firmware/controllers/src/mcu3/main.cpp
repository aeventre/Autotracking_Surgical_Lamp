#include "EncoderLib.h"
#include "StepperJoint.h"
#include "CommandParser.h"
#include <Arduino.h>

// —————————————————————————————————————————————————————————————
//  CHANGE THIS to whatever pin you’ve wired DE/RE on your RS-485 transceiver:
#define RS485_DIR 6
// —————————————————————————————————————————————————————————————

#define STEP_PIN 3
#define DIR_PIN  2

// your one stepper joint…
StepperJoint joint0;
// …and your encoder
EncoderLib encoder(Wire);
// the parser instance + a couple message structs
CommandParser cmdParser;
CommandMessage  rxMsg;
CommandMessage  txMsg;

unsigned long lastStatusMs = 0;

void setup() {
  // stepper driver pins
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // RS-485 DE/RE pin
  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW); // start in RX  

  // USB Serial for debug
  Serial.begin(115200);

  // I²C encoder on A4/A5 (UNO)
  encoder.begin(A4, A5);

  // attach encoder & pins to joint0, then zero & tune
  joint0.begin(STEP_PIN, DIR_PIN, &encoder, /*limit1=*/4, /*limit2=*/5);
  joint0.calibrateFromEncoder();
  joint0.setPIDGains(5, 1, 0.5);
  joint0.setTarget(0);

  // start RS-485 parser on Serial2 (RP2040)
  cmdParser.begin(Serial2);
}

void loop() {
  // 1) Check for new incoming command
  if (cmdParser.readCommand(rxMsg)) {
    // for now, we only have one joint → use a1
    joint0.setTarget(rxMsg.a1);
    cmdParser.clearReceivedFlag();
  }

  // 2) Step the joint controller forward
  joint0.update();

  // 3) Periodically send a status packet back
  if (millis() - lastStatusMs >= 100) {
    // fill txMsg with whatever you want to report
    txMsg.a1 = joint0.getAngle();  // current position
    txMsg.a2 = 0;
    txMsg.a3 = 0;
    txMsg.a4 = 0;
    txMsg.lightMode = 0;

    cmdParser.sendStatus(txMsg);
    lastStatusMs = millis();
  }
}