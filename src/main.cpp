#include <Arduino.h>

#include "LineFollower.h"
#include "Motors.h"
#include "RobotController.h"
#include "SerialTuning.h"
#include "WebControl.h"

void setup() {
  Serial.begin(115200);
  delay(300);

  beginMotors();
  beginLineFollower();
  stopMotors();
  beginRobotController();

  Serial.println("READY");
  beginWebControl();
}

void loop() {
  handleWebControl();
  handleSerialTuning();
  updateRobotController();
}
