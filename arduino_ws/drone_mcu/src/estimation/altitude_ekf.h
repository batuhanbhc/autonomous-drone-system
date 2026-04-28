#pragma once
#include <Arduino.h>

struct AltitudeEkfState {
  // EKF states
  float z_m        = 0.0f;   // vehicle world altitude
  float vz_mps     = 0.0f;   // vertical velocity
  float accelBias  = 0.0f;   // residual vertical accel bias
  float baroBias_m = 0.0f;   // baro drift / bias in meters

  // Floor tracker output (not part of EKF covariance state)
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
  uint8_t lidarFastRecoveryCount   = 0;
  uint8_t lidarRecoveryCount       = 0;
  uint8_t lidarRejectCount         = 0;
};

struct AltitudeEkf {
  AltitudeEkfState s;

  // Covariance P[4x4] for [z, vz, accelBias, baroBias]
  float P[4][4] = {};

  // Tuning
  float qAcc_mps2           = 5.0f;    // accel driving noise
  float qAccelBias          = 0.01f;   // accel bias RW
  float qBaroBias_m         = 0.005f;  // baro bias RW

  float rBaro_m             = 1.2f;    // baro stddev
  float maxPredictAccel_mps2 = 10.0f;  // reject impossible vertical accel spikes
  float maxBaroRate_mps      = 5.0f;  // reject implausible baro step-to-step jumps
  float maxBaroResidual_m    = 30.0f;  // hard cap on single baro innovation

  // Lidar validity limits live in sensors/lidar.h. Only keep EKF-specific tilt here.
  float maxTiltDeg          = 45.0f;

  // Surface tracking / acceptance logic.
  uint8_t recoverRejoinNeeded      = 2;
  float recoverRejoinBand_m        = 0.15f;
  uint8_t recoverConsecutiveNeeded = 20;
  uint8_t rejectConsecutiveNeeded  = 5;
  float recoverStableBand_m        = 0.30f;
  float reacquireGroundSum_m       = 0.0f;
  float reacquireGroundMin_m       = 0.0f;
  float reacquireGroundMax_m       = 0.0f;

  // Lidar statistical gate
  float maxLidarStd_m       = 2.5f;

  float mahaGateSigma       = 4.0f;

  // Internal bookkeeping
  bool   hasGroundEstimate    = false;
  bool   hasLastAcceptedLidar = false;
  float  lastLidarAccepted_m  = 0.0f;
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
