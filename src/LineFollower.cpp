#include "LineFollower.h"

#include <Arduino.h>

#include "HardwareConfig.h"
#include "RobotContext.h"

void beginLineFollower() {
  pinMode(hardware::lineSensor1, INPUT);
  pinMode(hardware::lineSensor2, INPUT);
  pinMode(hardware::lineSensor3, INPUT);
  pinMode(hardware::lineSensor4, INPUT);
  pinMode(hardware::lineSensor5, INPUT);
}

void updateLineFollower() {
  if (!robot.lineFollowerEnabled) {
    robot.lineTurn = 0;
    return;
  }

  const int s1 = digitalRead(hardware::lineSensor1);
  const int s2 = digitalRead(hardware::lineSensor2);
  const int s3 = digitalRead(hardware::lineSensor3);
  const int s4 = digitalRead(hardware::lineSensor4);
  const int s5 = digitalRead(hardware::lineSensor5);

  if (!s3 && s1 && s5) {
    robot.lineTurn = 0;
    robot.setpoint = robot.setpointBase - robot.driveSetpointOffset;
  } else if ((!s3 && !s1) || (!s1 && !s2 && !s3) || (!s1 && !s2)) {
    robot.lineTurn = -60;
    robot.setpoint = robot.setpointBase;
  } else if ((!s3 && !s5) || (!s5 && !s4 && !s3) || (!s5 && !s4)) {
    robot.lineTurn = 60;
    robot.setpoint = robot.setpointBase;
  } else if (!s2) {
    robot.lineTurn = -25;
    robot.setpoint = robot.setpointBase - 0.5f;
  } else if (!s1) {
    robot.lineTurn = -50;
    robot.setpoint = robot.setpointBase - 0.5f;
  } else if (!s4) {
    robot.lineTurn = 25;
    robot.setpoint = robot.setpointBase - 0.5f;
  } else if (!s5) {
    robot.lineTurn = 50;
    robot.setpoint = robot.setpointBase - 0.5f;
  } else {
    robot.lineTurn = 0;
    robot.setpoint = robot.setpointBase - robot.driveSetpointOffset;
  }
}
