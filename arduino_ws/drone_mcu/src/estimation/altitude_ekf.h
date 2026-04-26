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
  float agl_m               = 0.0f;  // z - groundZ
  bool  initialized         = false;

  // Diagnostics
  float lastBaroResidual_m         = 0.0f;
  float lastLidarResidual_m        = 0.0f;
  bool  lastLidarAccepted          = false;
  bool  lidarBlocked               = false;
  float lastImpliedGroundZ_m       = 0.0f;
  float lastGroundConsistencyErr_m = 0.0f;
  uint8_t lidarRecoveryCount       = 0;
};

struct AltitudeEkf {
  AltitudeEkfState s;

  // Covariance P[5x5]
  float P[5][5] = {};

  // Tuning
  float qAcc_mps2           = 2.5f;    // accel driving noise
  float qAccelBias          = 0.01f;   // accel bias RW
  float qBaroBias_m         = 0.005f;  // baro bias RW
  float qGround_m           = 0.0005f; // ground RW (slow)

  float rBaro_m             = 0.6f;    // baro stddev

  // Lidar validity limits
  float lidarMinRange_m     = 0.05f;
  float lidarMaxRange_m     = 20.0f;
  uint16_t lidarMinStrength = 200;
  float maxTiltDeg          = 45.0f;

  // One-sided obstacle / recovery logic.
  // Bands are combined with the lidar noise model so tuning stays minimal.
  float minAcceptBand_m     = 0.15f;   // normal acceptance band around locked floor
  float minObstacleBand_m   = 0.50f;   // upward floor jump => likely obstacle
  uint8_t recoverConsecutiveNeeded = 3;
  uint32_t minBlockHoldMs   = 150;     // chatter protection only
  uint32_t lidarBlockedSinceMs = 0;

  // Statistical gate
  float mahaGateSigma       = 3.5f;

  // Internal bookkeeping
  bool   hasLastAcceptedLidar = false;
  float  lastLidarAccepted_m  = 0.0f;
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