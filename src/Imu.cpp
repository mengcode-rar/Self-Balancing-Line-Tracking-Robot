#include "Imu.h"

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <math.h>

#include "HardwareConfig.h"
#include "RobotContext.h"

namespace {
MPU6050 mpu(Wire);
}

uint8_t beginImu() {
  Wire.begin(hardware::i2cSda, hardware::i2cScl);
  Wire.setClock(400000);
  return mpu.begin();
}

bool readImu(ImuSample& sample) {
  // Keep the slower I2C clock used by the original sketch during update().
  Wire.setClock(100000);
  mpu.update();
  Wire.setClock(400000);

  sample.accelerationX = mpu.getAccX();
  sample.accelerationZ = mpu.getAccZ();
  sample.gyroY = mpu.getGyroY();
  // MPU6050_light::update() does not report read failures.
  return true;
}

float accelerometerRoll(const ImuSample& sample) {
  return atan2f(-sample.accelerationX, sample.accelerationZ) * 180.0f / PI;
}

bool calibrateImu() {
  Serial.println("Calibrating... jangan gerak!");

  float gyroSum = 0.0f;
  float accelerationSum = 0.0f;
  int valid = 0;
  constexpr int sampleCount = 300;

  for (int i = 0; i < sampleCount; ++i) {
    ImuSample sample;
    if (readImu(sample)) {
      gyroSum += sample.gyroY;
      accelerationSum += accelerometerRoll(sample);
      ++valid;
    } else {
      Serial.println("Read failed!");
    }

    if (i % 50 == 0) {
      Serial.printf("step %d, valid=%d\n", i, valid);
    }
    delay(3);
  }

  Serial.printf("Loop selesai, valid=%d\n", valid);
  if (valid == 0) return false;

  robot.gyroOffsetY = gyroSum / valid;
  robot.accelerometerOffset = accelerationSum / valid;
  Serial.printf("gyroOffsetY=%.4f  accOffset=%.4f\n",
                robot.gyroOffsetY, robot.accelerometerOffset);
  return true;
}

float updateComplementaryFilter(float accelerationAngle, float gyroRate, float dt) {
  robot.filteredAccelerometerAngle =
      hardware::accelerometerLowPassAlpha * robot.filteredAccelerometerAngle +
      (1.0f - hardware::accelerometerLowPassAlpha) * accelerationAngle;

  robot.complementaryAngle =
      hardware::complementaryAlpha * (robot.complementaryAngle + gyroRate * dt) +
      (1.0f - hardware::complementaryAlpha) * robot.filteredAccelerometerAngle;
  return robot.complementaryAngle;
}

void resetComplementaryFilter(float angle) {
  robot.complementaryAngle = angle;
  robot.filteredAccelerometerAngle = angle;
}
