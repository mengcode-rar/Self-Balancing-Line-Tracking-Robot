#pragma once

#include <stdint.h>

enum class DriveCommand : uint8_t { Stop, Forward, Backward, Left, Right };
enum class RobotState : uint8_t { Boot, Init, Calibrate, Idle, Balancing, Fallen, Error };
enum class ErrorCode : uint8_t { None, ImuNotFound, ImuReadFail, CalibrationFail };

struct RobotContext {
  DriveCommand driveCommand = DriveCommand::Stop;
  float driveSetpointOffset = 1.2f;
  int turnOffset = 70;

  RobotState state = RobotState::Boot;
  ErrorCode error = ErrorCode::None;
  uint32_t lastControlUs = 0;

  float complementaryAngle = 0.0f;
  float filteredAccelerometerAngle = 0.0f;
  float rollDeg = 0.0f;
  float gyroOffsetY = 0.0f;
  float accelerometerOffset = 0.0f;

  float setpointBase = -0.89f;
  float setpoint = -0.89f;
  float kp = 30.0f;
  float ki = 300.0f;
  float kd = 0.8f;
  float pidIntegral = 0.0f;

  bool lineFollowerEnabled = true;
  int lineTurn = 0;
};

extern RobotContext robot;
