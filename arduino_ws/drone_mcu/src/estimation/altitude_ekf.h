#pragma once
#include <Arduino.h>

struct AltitudeEkfState {
  // EKF states
  float z_m        = 0.0f;   // vehicle world altitude
  float vz_mps     = 0.0f;   // vertical velocity
  float accelBias  = 0.0f;   // residual vertical accel bias
  float baroBias_m = 0.0f;   // baro drift / bias in meters

  // Outputs
  float agl_m               = 0.0f;  // alias of z for downstream compatibility
  bool  initialized         = false;

  // Diagnostics
  float lastBaroResidual_m   = 0.0f;
  float lastLidarResidual_m  = 0.0f;
  float lastRecoverySpread_m = 0.0f;
  bool  lastLidarAccepted    = false;
  bool  lidarBlocked         = false;
  uint8_t lidarRecoveryCount = 0;
  uint8_t lidarRejectCount   = 0;
};

struct AltitudeEkf {
  AltitudeEkfState s;

  // Covariance P[4x4] for [z, vz, accelBias, baroBias]
  float P[4][4] = {};

  // Tuning
  float qAcc_mps2           = 2.0f;    // accel driving noise
  float qAccelBias          = 0.01f;   // accel bias RW
  float qBaroBias_m         = 0.005f;  // baro bias RW

  float rBaro_m             = 0.3f;    // baro stddev
  float maxPredictAccel_mps2 = 10.0f;  // reject impossible vertical accel spikes
  float maxBaroRate_mps      = 2.5f;  // reject implausible baro step-to-step jumps
  float maxBaroResidual_m    = 10.0f;  // hard cap on single baro innovation

  // Max tilt degree for lidar vaalidity.
  float maxTiltDeg          = 45.0f;

  // Lidar outage / recovery logic.
  uint8_t rejectConsecutiveNeeded  = 5;
  uint8_t recoverConsecutiveNeeded = 6;
  float recoverStableBand_m        = 0.15f;
  float occlusionMinShort_m        = 0.25f;
  float occlusionMaxShort_m        = 0.60f;
  float occlusionSigma             = 3.0f;
  float recoverySigma              = 8.0f;
  float recoveryLidarSum_m         = 0.0f;
  float recoveryLidarMin_m         = 0.0f;
  float recoveryLidarMax_m         = 0.0f;

  // Lidar statistical gate
  float maxLidarStd_m       = 2.5f;

  float mahaGateSigma       = 4.0f;

  // Internal bookkeeping
  bool   hasLastAcceptedLidar  = false;
  float  lastLidarAccepted_m   = 0.0f;
  uint32_t lastLidarAcceptedMs = 0;
  bool   hasLastBaroSample    = false;
  float  lastBaroRel_m        = 0.0f;
  uint32_t lastBaroMs         = 0;
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
