#pragma once

#include <stdint.h>

namespace hardware {
constexpr uint8_t motorEnableA = 25;
constexpr uint8_t motorIn1 = 26;
constexpr uint8_t motorIn2 = 27;
constexpr uint8_t motorEnableB = 14;
constexpr uint8_t motorIn3 = 12;
constexpr uint8_t motorIn4 = 13;

constexpr uint8_t lineSensor1 = 33;
constexpr uint8_t lineSensor2 = 32;
constexpr uint8_t lineSensor3 = 35;
constexpr uint8_t lineSensor4 = 34;
constexpr uint8_t lineSensor5 = 15;

constexpr uint8_t i2cSda = 21;
constexpr uint8_t i2cScl = 22;

constexpr uint32_t pwmFrequency = 20000;
constexpr uint8_t pwmResolution = 8;
constexpr int dutyMax = 255;
constexpr int pwmSlew = 48;
constexpr uint32_t controlPeriodUs = 4000;

constexpr float complementaryAlpha = 0.98f;
constexpr float accelerometerLowPassAlpha = 0.85f;
}  // namespace hardware
