#include "RobotController.h"

#include <Arduino.h>
#include <math.h>

#include "HardwareConfig.h"
#include "Imu.h"
#include "LineFollower.h"
#include "Motors.h"
#include "RobotContext.h"

namespace {
void changeState(RobotState state) { robot.state = state; }

void stateBoot() {
  stopMotors();
  changeState(RobotState::Init);
}

void stateInit() {
  const uint8_t status = beginImu();
  if (status != 0) {
    Serial.printf("ERROR: MPU6050 tidak ditemukan! Status: %u\n", status);
    robot.error = ErrorCode::ImuNotFound;
    changeState(RobotState::Error);
    return;
  }

  ImuSample sample;
  if (!readImu(sample)) {
    Serial.println("ERROR: MPU6050 tidak bisa dibaca!");
    robot.error = ErrorCode::ImuReadFail;
    changeState(RobotState::Error);
    return;
  }

  Serial.println("IMU OK");
  changeState(RobotState::Calibrate);
}

void stateCalibrate() {
  if (!calibrateImu()) {
    robot.error = ErrorCode::CalibrationFail;
    changeState(RobotState::Error);
    return;
  }

  ImuSample sample;
  if (!readImu(sample)) {
    robot.error = ErrorCode::CalibrationFail;
    changeState(RobotState::Error);
    return;
  }

  const float roll = accelerometerRoll(sample);
  resetComplementaryFilter(roll);
  robot.rollDeg = roll;
  robot.pidIntegral = 0.0f;
  robot.setpoint = robot.setpointBase;

  Serial.println("Calibrate OK -> IDLE");
  Serial.println("Tegakkan robot...");
  changeState(RobotState::Idle);
}

void stateIdle() {
  static uint32_t lastUs = 0;
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - lastUs) < hardware::controlPeriodUs) return;
  lastUs = now;

  ImuSample sample;
  if (!readImu(sample)) {
    robot.error = ErrorCode::ImuReadFail;
    changeState(RobotState::Error);
    return;
  }

  robot.rollDeg = accelerometerRoll(sample);
  stopMotors();
  if (fabsf(robot.rollDeg) < 18.0f) {
    resetComplementaryFilter(robot.rollDeg);
    robot.pidIntegral = 0.0f;
    robot.setpoint = robot.setpointBase;
    robot.lastControlUs = micros();
    changeState(RobotState::Balancing);
    Serial.println("-> BALANCING");
  }
}

void stateBalancing() {
  const uint32_t now = micros();
  if (static_cast<uint32_t>(now - robot.lastControlUs) <
      hardware::controlPeriodUs) return;

  const float dt = static_cast<float>(now - robot.lastControlUs) * 1e-6f;
  robot.lastControlUs = now;

  ImuSample sample;
  if (!readImu(sample)) {
    stopMotors();
    robot.error = ErrorCode::ImuReadFail;
    changeState(RobotState::Error);
    return;
  }

  const float gyroRate = sample.gyroY - robot.gyroOffsetY;
  const float rollAcc = accelerometerRoll(sample);
  robot.rollDeg = updateComplementaryFilter(rollAcc, gyroRate, dt);

  if (fabsf(robot.rollDeg) > 50.0f) {
    stopMotors();
    robot.driveCommand = DriveCommand::Stop;
    robot.setpoint = robot.setpointBase;
    robot.pidIntegral = 0.0f;
    resetComplementaryFilter(robot.rollDeg);
    changeState(RobotState::Fallen);
    Serial.println("-> FALLEN");
    return;
  }

  float error = robot.setpoint - robot.rollDeg;
  if (fabsf(error) < 0.1f) error = 0.0f;

  robot.pidIntegral += error * dt;
  robot.pidIntegral = constrain(robot.pidIntegral, -5.0f, 5.0f);

  const float control = robot.kp * error + robot.ki * robot.pidIntegral -
                        robot.kd * gyroRate;
  const int balancePwm = controlToPwm(control, error);

  // The line sensor changes the next cycle's setpoint, as in the sketch.
  updateLineFollower();
  setMotors(-balancePwm);

  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 100) {
    lastPrintMs = millis();
    Serial.printf("roll=%.2f acc=%.2f sp=%.2f err=%.2f pwm=%d cmd=%d Kp=%.2f Ki=%.2f Kd=%.2f\n",
                  robot.rollDeg, rollAcc, robot.setpoint, error, -balancePwm,
                  static_cast<int>(robot.driveCommand), robot.kp, robot.ki,
                  robot.kd);
  }
}

void stateFallen() {
  static uint32_t lastUs = 0;
  const uint32_t now = micros();
  stopMotors();
  robot.driveCommand = DriveCommand::Stop;
  robot.setpoint = robot.setpointBase;
  if (static_cast<uint32_t>(now - lastUs) < hardware::controlPeriodUs) return;
  lastUs = now;

  ImuSample sample;
  if (!readImu(sample)) {
    robot.error = ErrorCode::ImuReadFail;
    changeState(RobotState::Error);
    return;
  }

  robot.rollDeg = accelerometerRoll(sample);
  if (fabsf(robot.rollDeg) < 18.0f) {
    resetComplementaryFilter(robot.rollDeg);
    robot.pidIntegral = 0.0f;
    changeState(RobotState::Idle);
    Serial.println("-> IDLE");
  }
}

void stateError() {
  stopMotors();
  robot.pidIntegral = 0.0f;

  static uint32_t lastMs = 0;
  if (millis() - lastMs >= 1000) {
    lastMs = millis();
    const char* messages[] = {
        "NONE", "IMU_NOT_FOUND", "IMU_READ_FAIL", "CALIB_FAIL"};
    Serial.printf("ERROR: %s\n", messages[static_cast<int>(robot.error)]);
  }
}
}  // namespace

void beginRobotController() { changeState(RobotState::Boot); }

void updateRobotController() {
  switch (robot.state) {
    case RobotState::Boot: stateBoot(); break;
    case RobotState::Init: stateInit(); break;
    case RobotState::Calibrate: stateCalibrate(); break;
    case RobotState::Idle: stateIdle(); break;
    case RobotState::Balancing: stateBalancing(); break;
    case RobotState::Fallen: stateFallen(); break;
    case RobotState::Error: stateError(); break;
    default:
      robot.error = ErrorCode::ImuNotFound;
      changeState(RobotState::Error);
      break;
  }
}
