#pragma once

void beginMotors();
void stopMotors();
void setMotors(int balancePwm);
int controlToPwm(float control, float error);
