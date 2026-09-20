#include "SerialTuning.h"

#include <Arduino.h>
#include <ctype.h>

#include "RobotContext.h"

void handleSerialTuning() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;

  const char type = toupper(static_cast<unsigned char>(command.charAt(0)));
  if (command.length() == 1) {
    if (type == 'X') {
      robot.pidIntegral = 0.0f;
      Serial.println(">> Integral reset");
    } else if (type == '?') {
      Serial.printf("Kp=%.4f Ki=%.4f Kd=%.4f sp_base=%.4f tilt=%.4f roll=%.2f\n",
                    robot.kp, robot.ki, robot.kd, robot.setpointBase,
                    robot.driveSetpointOffset, robot.rollDeg);
    }
    return;
  }

  const float value = command.substring(1).toFloat();
  switch (type) {
    case 'P':
      robot.kp = value;
      Serial.printf("Kp=%.4f\n", robot.kp);
      break;
    case 'I':
      robot.ki = value;
      robot.pidIntegral = 0.0f;
      Serial.printf("Ki=%.4f\n", robot.ki);
      break;
    case 'D':
      robot.kd = value;
      Serial.printf("Kd=%.4f\n", robot.kd);
      break;
    case 'S':
      robot.setpointBase = value;
      robot.setpoint = robot.setpointBase;
      Serial.printf("setpoint=%.4f\n", robot.setpointBase);
      break;
    case 'T':
      robot.driveSetpointOffset = value;
      Serial.printf("tilt=%.4f\n", robot.driveSetpointOffset);
      break;
    default:
      Serial.println(">> Tidak dikenal. Gunakan: P I D S T X ?");
      break;
  }
}
