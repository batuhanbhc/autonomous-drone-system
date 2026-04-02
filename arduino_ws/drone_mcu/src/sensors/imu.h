#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define IMU_INT_PIN        D1
#define IMU_RESET_PIN      -1
#define IMU_ADDR           0x4A
#define IMU_REPORT_RATE_US 5000  // 5ms = 200Hz

struct ImuData {
  float linAccel[3];
  float quat[4];
  float euler[3];      // from game rotation vector
  float gravEuler[2];  // [0]=pitch, [1]=roll from gravity vector (no yaw)
  bool  fresh;
};

extern ImuData imuData;

bool imuBegin();
void imuUpdate();