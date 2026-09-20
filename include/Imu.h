#pragma once

#include <stdint.h>

struct ImuSample {
  float accelerationX;
  float accelerationZ;
  float gyroY;
};

uint8_t beginImu();
bool readImu(ImuSample& sample);
bool calibrateImu();
float accelerometerRoll(const ImuSample& sample);
float updateComplementaryFilter(float accelerometerAngle, float gyroRate, float dt);
void resetComplementaryFilter(float angle);
