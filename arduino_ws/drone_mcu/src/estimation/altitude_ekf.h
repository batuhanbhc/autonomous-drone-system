#pragma once
#include <Arduino.h>

struct AltitudeEkfState {
  // Core states
  float z_m        = 0.0f;   // vehicle world altitude
  float vz_mps     = 0.0f;   // vertical velocity
  float accelBias  = 0.0f;   // residual vertical accel bias
  float baroBias_m = 0.0f;   // baro drift / bias in meters
  float groundZ_m  = 0.0f;   // local ground world altitude

  // Outputs
  float agl_m              = 0.0f;  // z - groundZ
  bool  initialized        = false;

  // Diagnostics
  float lastBaroResidual_m  = 0.0f;
  float lastLidarResidual_m = 0.0f;
  bool  lastLidarAccepted   = false;
};

struct AltitudeEkf {
  AltitudeEkfState s;

  // Covariance P[5x5]
  float P[5][5] = {};

  // Tuning
  float qAcc_mps2          = 3.0f;     // accel driving noise
  float qAccelBias         = 0.005f;   // accel bias RW
  float qBaroBias_m        = 0.002f;   // baro bias RW
  float qGround_m          = 0.001f;   // ground RW (slow)

  float rBaro_m            = 0.8f;     // baro stddev

  // Lidar acceptance parameters
  float lidarMinRange_m    = 0.05f;
  float lidarMaxRange_m    = 20.0f;
  uint16_t lidarMinStrength = 200;

  float maxTiltDeg         = 45.0f;
  float lidarJumpSlack_m   = 0.10f;
  float obstacleNegJump_m  = 0.25f;    // sudden shorter range threshold
  float mahaGateSigma      = 3.5f;

  // Obstacle cooldown
  uint32_t rejectCooldownMs = 200;
  uint32_t lidarRejectUntilMs = 0;

  // Internal bookkeeping
  bool   hasLastAcceptedLidar = false;
  float  lastLidarAccepted_m = 0.0f;
  uint32_t lastLidarAcceptedMs = 0;
};

extern AltitudeEkf altitudeEkf;

// Init / main API
void altitudeEkfReset();
void altitudeEkfInitialize(float initialBaroRel_m,
                           bool lidarValid,
                           float initialLidarAgl_m);
void altitudeEkfPredict(float accelWorldZ_mps2, float dt_s);
void altitudeEkfUpdateBaro(float baroRel_m);
bool altitudeEkfUpdateLidar(float lidarVertical_m,
                            uint16_t distanceCm,
                            uint16_t strength,
                            float r22_abs,
                            uint32_t nowMs);

// Helpers
float altitudeEkfGetAglM();
float altitudeEkfGetWorldZM();
float altitudeEkfGetGroundZM();
bool  altitudeEkfIsInitialized();