#include "altitude_ekf.h"
#include "../sensors/lidar.h"
#include <math.h>

AltitudeEkf altitudeEkf;

static constexpr int NX = 4;
enum {
  IX_Z  = 0,
  IX_VZ = 1,
  IX_BA = 2,
  IX_BB = 3
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

static float computeBaseLidarStdM(uint16_t distanceCm) {
  const float raw_m = 0.01f * distanceCm;

  if (raw_m <= 6.0f) {
    return 0.02f;
  }
  return 0.01f * raw_m;
}

static void resetLidarRecoveryWindow() {
  altitudeEkf.s.lidarFastRecoveryCount = 0;
  altitudeEkf.s.lidarRecoveryCount = 0;
  altitudeEkf.reacquireGroundSum_m = 0.0f;
  altitudeEkf.reacquireGroundMin_m = 0.0f;
  altitudeEkf.reacquireGroundMax_m = 0.0f;
}

static void beginLidarRecoveryWindow(float impliedGroundZ) {
  altitudeEkf.s.lidarRecoveryCount = 1;
  altitudeEkf.reacquireGroundSum_m = impliedGroundZ;
  altitudeEkf.reacquireGroundMin_m = impliedGroundZ;
  altitudeEkf.reacquireGroundMax_m = impliedGroundZ;
}

void altitudeEkfReset() {
  altitudeEkf = {};
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
    altitudeEkf.hasGroundEstimate = true;
    altitudeEkf.hasLastAcceptedLidar = true;
    altitudeEkf.lastLidarAccepted_m = initialLidarAgl_m;
    altitudeEkf.lastLidarAcceptedMs = millis();
  } else {
    altitudeEkf.s.groundZ_m = altitudeEkf.s.z_m;
    altitudeEkf.hasGroundEstimate = false;
    altitudeEkf.hasLastAcceptedLidar = false;
  }

  zeroMat(altitudeEkf.P);
  altitudeEkf.P[IX_Z][IX_Z]   = 1.0f;
  altitudeEkf.P[IX_VZ][IX_VZ] = 1.0f;
  altitudeEkf.P[IX_BA][IX_BA] = 1.0f;
  altitudeEkf.P[IX_BB][IX_BB] = 2.0f;

  altitudeEkf.s.initialized = true;
  altitudeEkf.s.lidarBlocked = !lidarValid;
  resetLidarRecoveryWindow();
  altitudeEkf.s.lidarRejectCount = 0;
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

  const bool lidarTrustedNow = altitudeEkf.hasGroundEstimate &&
      altitudeEkf.s.lastLidarAccepted &&
      !altitudeEkf.s.lidarBlocked;

  if (lidarTrustedNow) {
    rBaro *= 1.5f;
  }

  // Only let residual-driven baro downweighting happen while lidar is healthy.
  if (lidarTrustedNow) {
    baroResidAbsLPF = 0.95f * baroResidAbsLPF +
                      0.05f * fabsf(altitudeEkf.s.lastBaroResidual_m);
    rBaro += clampf((baroResidAbsLPF - 0.5f) * 1.5f, 0.0f, 3.0f);
  } else {
    baroResidAbsLPF = 0.0f;
  }

  rBaro = clampf(rBaro, altitudeEkf.rBaro_m, 6.0f);

  const float H[NX] = {1.0f, 0.0f, 0.0f, 1.0f};
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

  const bool validStrength = strength >= LIDAR_MIN_STRENGTH;
  const bool validRange =
      lidarVertical_m >= LIDAR_MIN_RANGE_M &&
      lidarVertical_m <= LIDAR_MAX_RANGE_M;
  const bool validTilt = r22_abs >= cosf(altitudeEkf.maxTiltDeg * DEG_TO_RAD);
  if (!(validStrength && validRange && validTilt)) {
    if (altitudeEkf.s.lidarBlocked) {
      resetLidarRecoveryWindow();
    }
    return false;
  }

  const float impliedGroundZ = altitudeEkf.s.z_m - lidarVertical_m;
  altitudeEkf.s.lastImpliedGroundZ_m = impliedGroundZ;

  float groundErr = 0.0f;
  if (altitudeEkf.hasGroundEstimate) {
    groundErr = impliedGroundZ - altitudeEkf.s.groundZ_m;
  }
  altitudeEkf.s.lastGroundConsistencyErr_m = groundErr;

  if (altitudeEkf.s.lidarBlocked) {
    const bool canFastRejoin =
        altitudeEkf.hasGroundEstimate &&
        fabsf(groundErr) <= altitudeEkf.recoverRejoinBand_m;

    if (canFastRejoin) {
      if (altitudeEkf.s.lidarFastRecoveryCount < 255) {
        altitudeEkf.s.lidarFastRecoveryCount++;
      }
      altitudeEkf.s.lidarRecoveryCount = 0;
      altitudeEkf.reacquireGroundSum_m = 0.0f;
      altitudeEkf.reacquireGroundMin_m = 0.0f;
      altitudeEkf.reacquireGroundMax_m = 0.0f;

      if (altitudeEkf.s.lidarFastRecoveryCount >= altitudeEkf.recoverRejoinNeeded) {
        altitudeEkf.s.lidarBlocked = false;
        altitudeEkf.s.lidarRejectCount = 0;
        resetLidarRecoveryWindow();
      } else {
        return false;
      }
    } else {
      altitudeEkf.s.lidarFastRecoveryCount = 0;

      if (altitudeEkf.s.lidarRecoveryCount == 0) {
        beginLidarRecoveryWindow(impliedGroundZ);
      } else {
        altitudeEkf.reacquireGroundSum_m += impliedGroundZ;
        altitudeEkf.reacquireGroundMin_m = fminf(altitudeEkf.reacquireGroundMin_m, impliedGroundZ);
        altitudeEkf.reacquireGroundMax_m = fmaxf(altitudeEkf.reacquireGroundMax_m, impliedGroundZ);
        if (altitudeEkf.s.lidarRecoveryCount < 255) altitudeEkf.s.lidarRecoveryCount++;
      }

      if (altitudeEkf.s.lidarRecoveryCount >= altitudeEkf.recoverConsecutiveNeeded) {
        const float spread = altitudeEkf.reacquireGroundMax_m - altitudeEkf.reacquireGroundMin_m;
        if (spread <= altitudeEkf.recoverStableBand_m) {
          const float sampleCount = (float)altitudeEkf.s.lidarRecoveryCount;
          altitudeEkf.s.groundZ_m = altitudeEkf.reacquireGroundSum_m / sampleCount;
          altitudeEkf.hasGroundEstimate = true;
          altitudeEkf.s.lidarBlocked = false;
          altitudeEkf.s.lidarRejectCount = 0;
          resetLidarRecoveryWindow();
          groundErr = impliedGroundZ - altitudeEkf.s.groundZ_m;
          altitudeEkf.s.lastGroundConsistencyErr_m = groundErr;
          updateDerivedOutputs();
        } else {
          beginLidarRecoveryWindow(impliedGroundZ);
          return false;
        }
      }

      if (altitudeEkf.s.lidarBlocked) return false;
    }
  }

  const float lidarStd = clampf(computeBaseLidarStdM(distanceCm),
                                0.0f,
                                altitudeEkf.maxLidarStd_m);
  const float R = sqf(lidarStd);
  const float zMeas = altitudeEkf.s.groundZ_m + lidarVertical_m;
  const float predZ = altitudeEkf.s.z_m;
  const float residual = zMeas - predZ;
  const float H[NX] = {1.0f, 0.0f, 0.0f, 0.0f};
  if (!lidarMahalanobisPass(H, residual, R)) {
    altitudeEkf.s.lastLidarResidual_m = residual;
    if (altitudeEkf.s.lidarRejectCount < 255) altitudeEkf.s.lidarRejectCount++;
    if (altitudeEkf.s.lidarRejectCount >= altitudeEkf.rejectConsecutiveNeeded) {
      altitudeEkf.s.lidarBlocked = true;
      resetLidarRecoveryWindow();
    }
    return false;
  }

  scalarUpdate(H, zMeas, predZ, R, &altitudeEkf.s.lastLidarResidual_m);

  altitudeEkf.s.lastLidarAccepted = true;
  altitudeEkf.s.lidarRejectCount = 0;
  altitudeEkf.hasLastAcceptedLidar = true;
  altitudeEkf.lastLidarAccepted_m = lidarVertical_m;
  altitudeEkf.lastLidarAcceptedMs = nowMs;

  return true;
}

float altitudeEkfGetAglM()              { return altitudeEkf.s.agl_m; }
float altitudeEkfGetWorldZM()           { return altitudeEkf.s.z_m; }
float altitudeEkfGetGroundZM()          { return altitudeEkf.s.groundZ_m; }
bool  altitudeEkfIsInitialized()        { return altitudeEkf.s.initialized; }
