#include "altitude_ekf.h"
#include "../sensors/lidar.h"
#include <math.h>

AltitudeEkf altitudeEkf;

static constexpr int NX = 5;
enum {
  IX_Z  = 0,
  IX_VZ = 1,
  IX_BA = 2,
  IX_BB = 3,
  IX_ZG = 4
};

static inline float sqf(float x) { return x * x; }
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static void zeroMat(float M[NX][NX]) {
  for (int r = 0; r < NX; ++r)
    for (int c = 0; c < NX; ++c)
      M[r][c] = 0.0f;
}

static void setIdentityScaled(float M[NX][NX], float d) {
  zeroMat(M);
  for (int i = 0; i < NX; ++i) M[i][i] = d;
}

static void copyMat(const float A[NX][NX], float B[NX][NX]) {
  for (int r = 0; r < NX; ++r)
    for (int c = 0; c < NX; ++c)
      B[r][c] = A[r][c];
}

static void mulMat(float A[NX][NX], float B[NX][NX], float C[NX][NX]) {
  float out[NX][NX] = {};
  for (int r = 0; r < NX; ++r) {
    for (int c = 0; c < NX; ++c) {
      float s = 0.0f;
      for (int k = 0; k < NX; ++k) s += A[r][k] * B[k][c];
      out[r][c] = s;
    }
  }
  copyMat(out, C);
}

static void transposeMat(float A[NX][NX], float AT[NX][NX]) {
  for (int r = 0; r < NX; ++r)
    for (int c = 0; c < NX; ++c)
      AT[r][c] = A[c][r];
}

static void addMatInPlace(float A[NX][NX], float B[NX][NX]) {
  for (int r = 0; r < NX; ++r)
    for (int c = 0; c < NX; ++c)
      A[r][c] += B[r][c];
}

static void symmetrize(float P[NX][NX]) {
  for (int r = 0; r < NX; ++r) {
    for (int c = r + 1; c < NX; ++c) {
      const float s = 0.5f * (P[r][c] + P[c][r]);
      P[r][c] = s;
      P[c][r] = s;
    }
  }
}

static void updateDerivedOutputs() {
  altitudeEkf.s.agl_m = altitudeEkf.s.z_m - altitudeEkf.s.groundZ_m;
}

static float computeLidarStdM(uint16_t distanceCm, float r22_abs) {
  const float raw_m = 0.01f * distanceCm;

  float slantStd;
  if (raw_m <= 6.0f) {
    slantStd = 0.02f;
  } else {
    slantStd = 0.005f * raw_m;
  }

  const float r22_safe = fmaxf(r22_abs, cosf(altitudeEkf.maxTiltDeg * DEG_TO_RAD));
  return slantStd / r22_safe;
}

void altitudeEkfReset() {
  altitudeEkf = {};
  altitudeEkf.lidarMinRange_m = LIDAR_MIN_RANGE_M;
  altitudeEkf.lidarMaxRange_m = LIDAR_MAX_RANGE_M;
  altitudeEkf.lidarMinStrength = LIDAR_MIN_STRENGTH;
  setIdentityScaled(altitudeEkf.P, 10.0f);
}

void altitudeEkfInitialize(float initialBaroRel_m,
                           bool lidarValid,
                           float initialLidarAgl_m) {
  altitudeEkfReset();

  altitudeEkf.s.z_m        = initialBaroRel_m;
  altitudeEkf.s.vz_mps     = 0.0f;
  altitudeEkf.s.accelBias  = 0.0f;
  altitudeEkf.s.baroBias_m = 0.0f;

  if (lidarValid) {
    altitudeEkf.s.groundZ_m = altitudeEkf.s.z_m - initialLidarAgl_m;
    altitudeEkf.hasLastAcceptedLidar = true;
    altitudeEkf.lastLidarAccepted_m = initialLidarAgl_m;
    altitudeEkf.lastLidarAcceptedMs = millis();
  } else {
    altitudeEkf.s.groundZ_m = altitudeEkf.s.z_m;
    altitudeEkf.hasLastAcceptedLidar = false;
  }

  zeroMat(altitudeEkf.P);
  altitudeEkf.P[IX_Z][IX_Z]   = 1.0f;
  altitudeEkf.P[IX_VZ][IX_VZ] = 1.0f;
  altitudeEkf.P[IX_BA][IX_BA] = 1.0f;
  altitudeEkf.P[IX_BB][IX_BB] = 2.0f;
  altitudeEkf.P[IX_ZG][IX_ZG] = lidarValid ? 0.5f : 4.0f;

  altitudeEkf.s.initialized = true;
  altitudeEkf.s.lidarBlocked = false;
  altitudeEkf.s.lidarRecoveryCount = 0;
  altitudeEkf.s.lastImpliedGroundZ_m = altitudeEkf.s.groundZ_m;
  altitudeEkf.s.lastGroundConsistencyErr_m = 0.0f;
  updateDerivedOutputs();
}

void altitudeEkfPredict(float accelWorldZ_mps2, float dt_s) {
  if (!altitudeEkf.s.initialized) return;
  if (!(dt_s > 0.0f) || dt_s > 0.1f) return;

  const float a = accelWorldZ_mps2 - altitudeEkf.s.accelBias;

  altitudeEkf.s.z_m    += altitudeEkf.s.vz_mps * dt_s + 0.5f * a * dt_s * dt_s;
  altitudeEkf.s.vz_mps += a * dt_s;

  float F[NX][NX] = {};
  for (int i = 0; i < NX; ++i) F[i][i] = 1.0f;
  F[IX_Z][IX_VZ]  = dt_s;
  F[IX_Z][IX_BA]  = -0.5f * dt_s * dt_s;
  F[IX_VZ][IX_BA] = -dt_s;

  float Q[NX][NX] = {};
  const float dt2 = dt_s * dt_s;
  const float dt3 = dt2 * dt_s;
  const float dt4 = dt2 * dt2;

  const float qA = sqf(altitudeEkf.qAcc_mps2);
  Q[IX_Z][IX_Z]   = 0.25f * dt4 * qA;
  Q[IX_Z][IX_VZ]  = 0.5f  * dt3 * qA;
  Q[IX_VZ][IX_Z]  = 0.5f  * dt3 * qA;
  Q[IX_VZ][IX_VZ] = dt2 * qA;

  Q[IX_BA][IX_BA] = dt_s * sqf(altitudeEkf.qAccelBias);
  Q[IX_BB][IX_BB] = dt_s * sqf(altitudeEkf.qBaroBias_m);
  Q[IX_ZG][IX_ZG] = dt_s * sqf(altitudeEkf.qGround_m);

  float FP[NX][NX], FT[NX][NX], FPFt[NX][NX];
  mulMat(F, altitudeEkf.P, FP);
  transposeMat(F, FT);
  mulMat(FP, FT, FPFt);
  copyMat(FPFt, altitudeEkf.P);
  addMatInPlace(altitudeEkf.P, Q);
  symmetrize(altitudeEkf.P);

  updateDerivedOutputs();
}

static void scalarUpdate(const float H[NX], float meas, float pred, float R, float* residualOut) {
  float PHt[NX] = {};
  for (int r = 0; r < NX; ++r) {
    float s = 0.0f;
    for (int c = 0; c < NX; ++c) s += altitudeEkf.P[r][c] * H[c];
    PHt[r] = s;
  }

  float S = R;
  for (int i = 0; i < NX; ++i) S += H[i] * PHt[i];
  if (S < 1e-6f) return;

  const float residual = meas - pred;
  if (residualOut) *residualOut = residual;

  float K[NX];
  for (int i = 0; i < NX; ++i) K[i] = PHt[i] / S;

  altitudeEkf.s.z_m        += K[IX_Z]  * residual;
  altitudeEkf.s.vz_mps     += K[IX_VZ] * residual;
  altitudeEkf.s.accelBias  += K[IX_BA] * residual;
  altitudeEkf.s.baroBias_m += K[IX_BB] * residual;
  altitudeEkf.s.groundZ_m  += K[IX_ZG] * residual;

  float I_KH[NX][NX] = {};
  for (int r = 0; r < NX; ++r) {
    for (int c = 0; c < NX; ++c) {
      I_KH[r][c] = (r == c ? 1.0f : 0.0f) - K[r] * H[c];
    }
  }

  float tmp[NX][NX];
  float I_KH_T[NX][NX];
  float joseph[NX][NX];

  mulMat(I_KH, altitudeEkf.P, tmp);
  transposeMat(I_KH, I_KH_T);
  mulMat(tmp, I_KH_T, joseph);

  for (int r = 0; r < NX; ++r) {
    for (int c = 0; c < NX; ++c) {
      joseph[r][c] += K[r] * R * K[c];
    }
  }

  copyMat(joseph, altitudeEkf.P);
  symmetrize(altitudeEkf.P);
  updateDerivedOutputs();
}

void altitudeEkfUpdateBaro(float baroRel_m) {
  if (!altitudeEkf.s.initialized) return;

  static float baroResidAbsLPF = 0.0f;

  float rBaro = altitudeEkf.rBaro_m;

  // Simple rule: when lidar is trusted and not currently blocked, let baro
  // have less authority. This replaces the previous altitude-shaped heuristic.
  const bool lidarTrustedRecent = altitudeEkf.hasLastAcceptedLidar &&
      !altitudeEkf.s.lidarBlocked &&
      (millis() - altitudeEkf.lastLidarAcceptedMs) < 500;

  if (lidarTrustedRecent) {
    rBaro *= 3.0f;
  }

  // Inflate baro noise a bit more when recent baro residuals stay large.
  baroResidAbsLPF = 0.95f * baroResidAbsLPF +
                    0.05f * fabsf(altitudeEkf.s.lastBaroResidual_m);

  rBaro += clampf((baroResidAbsLPF - 0.5f) * 1.5f, 0.0f, 3.0f);
  rBaro = clampf(rBaro, altitudeEkf.rBaro_m, 6.0f);

  const float H[NX] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  const float pred = altitudeEkf.s.z_m + altitudeEkf.s.baroBias_m;
  scalarUpdate(H, baroRel_m, pred, sqf(rBaro),
               &altitudeEkf.s.lastBaroResidual_m);
}

static bool lidarMahalanobisPass(const float H[NX], float residual, float R) {
  float PHt[NX] = {};
  for (int r = 0; r < NX; ++r) {
    float s = 0.0f;
    for (int c = 0; c < NX; ++c) s += altitudeEkf.P[r][c] * H[c];
    PHt[r] = s;
  }

  float S = R;
  for (int i = 0; i < NX; ++i) S += H[i] * PHt[i];
  if (S < 1e-6f) return false;

  const float d2 = (residual * residual) / S;
  return d2 <= sqf(altitudeEkf.mahaGateSigma);
}

bool altitudeEkfUpdateLidar(float lidarVertical_m,
                            uint16_t distanceCm,
                            uint16_t strength,
                            float r22_abs,
                            uint32_t nowMs) {
  altitudeEkf.s.lastLidarAccepted = false;
  if (!altitudeEkf.s.initialized) return false;

  if (strength < altitudeEkf.lidarMinStrength) return false;
  if (!(lidarVertical_m >= altitudeEkf.lidarMinRange_m && lidarVertical_m <= altitudeEkf.lidarMaxRange_m)) return false;
  if (r22_abs < cosf(altitudeEkf.maxTiltDeg * DEG_TO_RAD)) return false;

  const float predAgl = altitudeEkf.s.z_m - altitudeEkf.s.groundZ_m;
  const float residual = lidarVertical_m - predAgl;
  const float lidarStd = computeLidarStdM(distanceCm, r22_abs);
  const float acceptBand = fmaxf(altitudeEkf.minAcceptBand_m, 3.0f * lidarStd);
  const float obstacleBand = fmaxf(altitudeEkf.minObstacleBand_m, 5.0f * lidarStd);

  const float impliedGroundZ = altitudeEkf.s.z_m - lidarVertical_m;
  const float groundErr = impliedGroundZ - altitudeEkf.s.groundZ_m;
  altitudeEkf.s.lastImpliedGroundZ_m = impliedGroundZ;
  altitudeEkf.s.lastGroundConsistencyErr_m = groundErr;

  // If the implied ground suddenly moves upward, the beam likely hit an obstacle.
  if (!altitudeEkf.s.lidarBlocked && groundErr > obstacleBand) {
    altitudeEkf.s.lidarBlocked = true;
    altitudeEkf.s.lidarRecoveryCount = 0;
    altitudeEkf.lidarBlockedSinceMs = nowMs;
    return false;
  }

  if (altitudeEkf.s.lidarBlocked) {
    const bool minHoldDone = (uint32_t)(nowMs - altitudeEkf.lidarBlockedSinceMs) >= altitudeEkf.minBlockHoldMs;
    if (minHoldDone && fabsf(groundErr) <= acceptBand) {
      if (altitudeEkf.s.lidarRecoveryCount < 255) altitudeEkf.s.lidarRecoveryCount++;
      if (altitudeEkf.s.lidarRecoveryCount >= altitudeEkf.recoverConsecutiveNeeded) {
        altitudeEkf.s.lidarBlocked = false;
        altitudeEkf.s.lidarRecoveryCount = 0;
      }
    } else {
      altitudeEkf.s.lidarRecoveryCount = 0;
    }

    if (altitudeEkf.s.lidarBlocked) {
      return false;
    }
  }

  // In normal mode, require floor consistency before giving the EKF a chance to update.
  if (fabsf(groundErr) > acceptBand) {
    return false;
  }

  const float R = sqf(lidarStd);
  const float H[NX] = {1.0f, 0.0f, 0.0f, 0.0f, -1.0f};
  if (!lidarMahalanobisPass(H, residual, R)) {
    return false;
  }

  scalarUpdate(H, lidarVertical_m, predAgl, R, &altitudeEkf.s.lastLidarResidual_m);

  altitudeEkf.s.lastLidarAccepted = true;
  altitudeEkf.hasLastAcceptedLidar = true;
  altitudeEkf.lastLidarAccepted_m = lidarVertical_m;
  altitudeEkf.lastLidarAcceptedMs = nowMs;

  return true;
}

float altitudeEkfGetAglM()              { return altitudeEkf.s.agl_m; }
float altitudeEkfGetWorldZM()           { return altitudeEkf.s.z_m; }
float altitudeEkfGetGroundZM()          { return altitudeEkf.s.groundZ_m; }
bool  altitudeEkfIsInitialized()        { return altitudeEkf.s.initialized; }