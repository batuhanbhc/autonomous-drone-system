#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define IMU_INT_PIN           D1
#define IMU_RESET_PIN        -1
#define IMU_ADDR              0x4A
#define IMU_REPORT_RATE_US    5000 // ~200 hz 

#define IMU_OFFSET_X_CM      -6.0f
#define IMU_OFFSET_Y_CM       3.0f
#define IMU_YAW_OFFSET_DEG   -135.0f  // IMU to drone, + is cw, - is ccw


struct ImuMounting {
  float xOffset_cm = 0.0f;
  float yOffset_cm = 0.0f;
  float yawOffset_deg = 0.0f;
};

struct ImuBias {
  float droneLinAccel[3] = {0.0f, 0.0f, 0.0f}; // m/s^2 bias in drone/body frame
  float droneGyro[3] = {0.0f, 0.0f, 0.0f};
};

struct ImuData {
  // raw imu
  float imuLinAccel[3]   = {0};
  float imuGyro[3]       = {0};
  float imuQuat[4]       = {0,0,0,1};

  float levelCorrQuat[4] = {0.0f, 0.0f, 0.0f, 1.0f};  // q_corr
  bool  levelCorrValid   = false;

  float droneLinAccel[3] = {0};
  float droneGyro[3]     = {0};
  float droneQuat[4]     = {0,0,0,1};
  float worldLinAccelZ   = 0.0f;

  uint64_t timestamp_us  = 0;
  uint32_t dt_us         = 0;
  bool fresh             = false;

  ImuMounting mount;
  ImuBias bias;

  bool initialized       = false;
};

extern ImuData imuData;

void imuSetMounting();
bool imuBegin();
void imuUpdate();
