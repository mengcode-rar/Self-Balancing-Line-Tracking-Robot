#include "Motors.h"

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>

#include "HardwareConfig.h"
#include "RobotContext.h"

namespace {
int lastPwmA = 0;
int lastPwmB = 0;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t motorChannelA = 0;
constexpr uint8_t motorChannelB = 1;
#endif

void writePwmA(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(hardware::motorEnableA, duty);
#else
  ledcWrite(motorChannelA, duty);
#endif
}

void writePwmB(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(hardware::motorEnableB, duty);
#else
  ledcWrite(motorChannelB, duty);
#endif
}

void setMotorA(int pwm) {
  pwm = constrain(pwm, -hardware::dutyMax, hardware::dutyMax);
  digitalWrite(hardware::motorIn1, pwm < 0 ? HIGH : LOW);
  digitalWrite(hardware::motorIn2, pwm > 0 ? HIGH : LOW);
  writePwmA(abs(pwm));
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -hardware::dutyMax, hardware::dutyMax);
  digitalWrite(hardware::motorIn3, pwm > 0 ? HIGH : LOW);
  digitalWrite(hardware::motorIn4, pwm < 0 ? HIGH : LOW);
  writePwmB(abs(pwm));
}

int slew(int target, int& previous) {
  previous = constrain(target, previous - hardware::pwmSlew,
                       previous + hardware::pwmSlew);
  return previous;
}
}  // namespace

void beginMotors() {
  pinMode(hardware::motorIn1, OUTPUT);
  pinMode(hardware::motorIn2, OUTPUT);
  pinMode(hardware::motorIn3, OUTPUT);
  pinMode(hardware::motorIn4, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(hardware::motorEnableA, hardware::pwmFrequency,
             hardware::pwmResolution);
  ledcAttach(hardware::motorEnableB, hardware::pwmFrequency,
             hardware::pwmResolution);
#else
  ledcSetup(motorChannelA, hardware::pwmFrequency, hardware::pwmResolution);
  ledcSetup(motorChannelB, hardware::pwmFrequency, hardware::pwmResolution);
  ledcAttachPin(hardware::motorEnableA, motorChannelA);
  ledcAttachPin(hardware::motorEnableB, motorChannelB);
#endif
}

void stopMotors() {
  lastPwmA = 0;
  lastPwmB = 0;
  setMotorA(0);
  setMotorB(0);
}

void setMotors(int balancePwm) {
  int offsetA = robot.lineTurn;
  int offsetB = -robot.lineTurn;

  if (robot.driveCommand == DriveCommand::Left) {
    offsetA -= robot.turnOffset;
    offsetB += robot.turnOffset;
  } else if (robot.driveCommand == DriveCommand::Right) {
    offsetA += robot.turnOffset;
    offsetB -= robot.turnOffset;
  }

  const int targetA = constrain(balancePwm + offsetA,
                                -hardware::dutyMax, hardware::dutyMax);
  const int targetB = constrain(balancePwm + offsetB,
                                -hardware::dutyMax, hardware::dutyMax);
  setMotorA(slew(targetA, lastPwmA));
  setMotorB(slew(targetB, lastPwmB));
}

int controlToPwm(float control, float error) {
  constexpr float errorNearZero = 0.15f;
  constexpr int minimumPwm = 160;
  if (fabsf(error) < errorNearZero) return 0;

  int pwm = static_cast<int>(control);
  if (pwm > 0 && pwm < minimumPwm) pwm = minimumPwm;
  if (pwm < 0 && pwm > -minimumPwm) pwm = -minimumPwm;
  return constrain(pwm, -hardware::dutyMax, hardware::dutyMax);
}
