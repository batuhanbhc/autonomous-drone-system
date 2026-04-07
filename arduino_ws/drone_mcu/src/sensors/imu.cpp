#include "imu.h"

static Adafruit_BNO08x  bno08x(IMU_RESET_PIN);
static sh2_SensorValue_t sensorValue;

ImuData imuData = {};

static volatile bool imuInterruptFired = false;
static uint32_t lastAccelRxUs = 0;
static float prevDroneGyro[3] = {0.0f, 0.0f, 0.0f};
static bool  prevDroneGyroValid = false;

static void quatNormalize(float q[4]);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void IRAM_ATTR imuISR() {
  imuInterruptFired = true;
}

static inline float deg2rad(float deg) {
  return deg * DEG_TO_RAD;
}

static void vecCopy3(const float in[3], float out[3]) {
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

static void matVecMul3(const float M[3][3], const float v[3], float out[3]) {
  out[0] = M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2];
  out[1] = M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2];
  out[2] = M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2];
}

static void matMul3(const float A[3][3], const float B[3][3], float C[3][3]) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      C[r][c] = A[r][0] * B[0][c] +
                A[r][1] * B[1][c] +
                A[r][2] * B[2][c];
    }
  }
}

static void matTranspose3(const float A[3][3], float AT[3][3]) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      AT[r][c] = A[c][r];
    }
  }
}

static void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// imu.cpp — add these functions

static void quatMul(const float a[4], const float b[4], float out[4]) {
  // Hamilton product, q = [x,y,z,w]
  out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
  out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
  out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
  out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

// Call once after the first stable IMU reading.
// Computes q_corr that levels roll/pitch while preserving yaw.
void imuComputeLevelCorrection() {
  const float* q = imuData.droneQuat;  // [x,y,z,w]

  // Extract yaw from current quaternion
  const float yaw = atan2f(
    2.0f * (q[3]*q[2] + q[0]*q[1]),
    1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])
  );

  // q_level = pure yaw quaternion (zero roll, zero pitch)
  const float q_level[4] = {
    0.0f,
    0.0f,
    sinf(yaw * 0.5f),
    cosf(yaw * 0.5f)
  };

  // q_corr = q_level * q_drone_inv
  // q_drone_inv = [-x, -y, -z, w] (unit quaternion conjugate)
  const float q_inv[4] = { -q[0], -q[1], -q[2], q[3] };

  float corr[4];
  // Hamilton product: q_level * q_inv
  corr[0] = q_level[3]*q_inv[0] + q_level[0]*q_inv[3] + q_level[1]*q_inv[2] - q_level[2]*q_inv[1];
  corr[1] = q_level[3]*q_inv[1] - q_level[0]*q_inv[2] + q_level[1]*q_inv[3] + q_level[2]*q_inv[0];
  corr[2] = q_level[3]*q_inv[2] + q_level[0]*q_inv[1] - q_level[1]*q_inv[0] + q_level[2]*q_inv[3];
  corr[3] = q_level[3]*q_inv[3] - q_level[0]*q_inv[0] - q_level[1]*q_inv[1] - q_level[2]*q_inv[2];

  // Normalize and store
  const float n = sqrtf(corr[0]*corr[0] + corr[1]*corr[1] + corr[2]*corr[2] + corr[3]*corr[3]);
  imuData.levelCorrQuat[0] = corr[0] / n;
  imuData.levelCorrQuat[1] = corr[1] / n;
  imuData.levelCorrQuat[2] = corr[2] / n;
  imuData.levelCorrQuat[3] = corr[3] / n;

  imuData.levelCorrValid = true;
}

void applyLevelCorrection() {
  if (!imuData.levelCorrValid) return;
  float corrected[4];
  quatMul(imuData.levelCorrQuat, imuData.droneQuat, corrected);
  // normalize
  const float n = sqrtf(corrected[0]*corrected[0] + corrected[1]*corrected[1] +
                         corrected[2]*corrected[2] + corrected[3]*corrected[3]);
  imuData.droneQuat[0] = corrected[0] / n;
  imuData.droneQuat[1] = corrected[1] / n;
  imuData.droneQuat[2] = corrected[2] / n;
  imuData.droneQuat[3] = corrected[3] / n;
}

static void rotationZDeg(float yawDeg, float R[3][3]) {
  const float c = cosf(deg2rad(yawDeg));
  const float s = sinf(deg2rad(yawDeg));

  // Maps vector components from IMU frame to DRONE frame.
  // "yawOffset_deg rotates IMU +Y onto drone +Y", CCW positive.
  R[0][0] =  c; R[0][1] = -s; R[0][2] = 0.0f;
  R[1][0] =  s; R[1][1] =  c; R[1][2] = 0.0f;
  R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
}

static void quatNormalize(float q[4]) {
  const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n <= 0.0f) {
    q[0] = q[1] = q[2] = 0.0f;
    q[3] = 1.0f;
    return;
  }
  q[0] /= n;
  q[1] /= n;
  q[2] /= n;
  q[3] /= n;
}

static void quatToMatrix(const float qIn[4], float R[3][3]) {
  float q[4] = { qIn[0], qIn[1], qIn[2], qIn[3] };
  quatNormalize(q);

  const float x = q[0];
  const float y = q[1];
  const float z = q[2];
  const float w = q[3];

  // Rotation matrix: IMU/body frame -> world frame
  R[0][0] = 1.0f - 2.0f * (y * y + z * z);
  R[0][1] = 2.0f * (x * y - z * w);
  R[0][2] = 2.0f * (x * z + y * w);

  R[1][0] = 2.0f * (x * y + z * w);
  R[1][1] = 1.0f - 2.0f * (x * x + z * z);
  R[1][2] = 2.0f * (y * z - x * w);

  R[2][0] = 2.0f * (x * z - y * w);
  R[2][1] = 2.0f * (y * z + x * w);
  R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void matrixToQuat(const float R[3][3], float q[4]) {
  const float trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0.0f) {
    const float s = sqrtf(trace + 1.0f) * 2.0f;
    q[3] = 0.25f * s;
    q[0] = (R[2][1] - R[1][2]) / s;
    q[1] = (R[0][2] - R[2][0]) / s;
    q[2] = (R[1][0] - R[0][1]) / s;
  } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
    const float s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
    q[3] = (R[2][1] - R[1][2]) / s;
    q[0] = 0.25f * s;
    q[1] = (R[0][1] + R[1][0]) / s;
    q[2] = (R[0][2] + R[2][0]) / s;
  } else if (R[1][1] > R[2][2]) {
    const float s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
    q[3] = (R[0][2] - R[2][0]) / s;
    q[0] = (R[0][1] + R[1][0]) / s;
    q[1] = 0.25f * s;
    q[2] = (R[1][2] + R[2][1]) / s;
  } else {
    const float s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
    q[3] = (R[1][0] - R[0][1]) / s;
    q[0] = (R[0][2] + R[2][0]) / s;
    q[1] = (R[1][2] + R[2][1]) / s;
    q[2] = 0.25f * s;
  }

  quatNormalize(q);
}

static bool setReports() {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION,  IMU_REPORT_RATE_US)) return false;
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_RATE_US)) return false;
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_REPORT_RATE_US)) return false;
  return true;
}

static void recomputeDerivedState() {
  // R_BI : IMU -> Drone
  float R_BI[3][3];
  rotationZDeg(imuData.mount.yawOffset_deg, R_BI);

  // R_IB : Drone -> IMU
  float R_IB[3][3];
  matTranspose3(R_BI, R_IB);

  // Rotate IMU-frame accel and gyro into drone body axes
  float accelAtImu_body[3];
  float gyro_body[3];

  matVecMul3(R_BI, imuData.imuLinAccel, accelAtImu_body);
  matVecMul3(R_BI, imuData.imuGyro,     gyro_body);

  vecCopy3(gyro_body, imuData.droneGyro);

  if (imuData.initialized) {
    imuData.droneGyro[0] -= imuData.bias.droneGyro[0];
    imuData.droneGyro[1] -= imuData.bias.droneGyro[1];
    imuData.droneGyro[2] -= imuData.bias.droneGyro[2];
  }

  // Offset correction to drone center
  const float r_body[3] = {
    imuData.mount.xOffset_cm * 0.01f,
    imuData.mount.yOffset_cm * 0.01f,
    0.0f
  };

  float alpha_body[3] = {0.0f, 0.0f, 0.0f};
  if (prevDroneGyroValid && imuData.dt_us > 0) {
    const float dt_s = 1e-6f * (float)imuData.dt_us;
    if (dt_s > 0.0f) {
      alpha_body[0] = (imuData.droneGyro[0] - prevDroneGyro[0]) / dt_s;
      alpha_body[1] = (imuData.droneGyro[1] - prevDroneGyro[1]) / dt_s;
      alpha_body[2] = (imuData.droneGyro[2] - prevDroneGyro[2]) / dt_s;
    }
  }

  float alphaCrossR[3];
  float omegaCrossR[3];
  float omegaCrossOmegaCrossR[3];

  cross3(alpha_body,        r_body, alphaCrossR);
  cross3(imuData.droneGyro, r_body, omegaCrossR);
  cross3(imuData.droneGyro, omegaCrossR, omegaCrossOmegaCrossR);

  float droneLinAccelRaw[3];
  droneLinAccelRaw[0] = accelAtImu_body[0] - alphaCrossR[0] - omegaCrossOmegaCrossR[0];
  droneLinAccelRaw[1] = accelAtImu_body[1] - alphaCrossR[1] - omegaCrossOmegaCrossR[1];
  droneLinAccelRaw[2] = accelAtImu_body[2] - alphaCrossR[2] - omegaCrossOmegaCrossR[2];

  prevDroneGyro[0] = imuData.droneGyro[0];
  prevDroneGyro[1] = imuData.droneGyro[1];
  prevDroneGyro[2] = imuData.droneGyro[2];
  prevDroneGyroValid = true;

  // Orientation
  float R_WI[3][3];
  float R_WB[3][3];

  quatToMatrix(imuData.imuQuat, R_WI);
  matMul3(R_WI, R_IB, R_WB);

  matrixToQuat(R_WB, imuData.droneQuat);

  applyLevelCorrection();

  if (imuData.initialized) {
    imuData.droneLinAccel[0] = droneLinAccelRaw[0] - imuData.bias.droneLinAccel[0];
    imuData.droneLinAccel[1] = droneLinAccelRaw[1] - imuData.bias.droneLinAccel[1];
    imuData.droneLinAccel[2] = droneLinAccelRaw[2] - imuData.bias.droneLinAccel[2];
  } else {
    imuData.droneLinAccel[0] = droneLinAccelRaw[0];
    imuData.droneLinAccel[1] = droneLinAccelRaw[1];
    imuData.droneLinAccel[2] = droneLinAccelRaw[2];
  }

  float worldAccel[3];
  matVecMul3(R_WB, imuData.droneLinAccel, worldAccel);
  imuData.worldLinAccelZ = worldAccel[2];
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void imuSetMounting() {
  imuData.mount.xOffset_cm = IMU_OFFSET_X_CM;
  imuData.mount.yOffset_cm = IMU_OFFSET_Y_CM;
  imuData.mount.yawOffset_deg = IMU_YAW_OFFSET_DEG;
}

bool imuBegin() {
  imuSetMounting();

  delay(200);
  Wire.begin(D4, D5);

  if (!bno08x.begin_I2C(IMU_ADDR, &Wire)) {
    Serial.println("[IMU] FATAL: Not detected on I2C!");
    return false;
  }

  Wire.setClock(400000);

  if (!setReports()) {
    Serial.println("[IMU] FATAL: Failed to set reports!");
    return false;
  }

  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuISR, FALLING);

  imuData.imuQuat[3]   = 1.0f;
  imuData.droneQuat[3] = 1.0f;

  return true;
}

void imuUpdate() {
  const uint32_t nowUs = micros();

  if (bno08x.wasReset()) {
    if (!setReports()) {
      imuData.fresh = false;
      return;
    }
  }

  if (!imuInterruptFired) return;
  imuInterruptFired = false;

  bool gotAny = false;
  bool gotAccel = false;

  while (bno08x.getSensorEvent(&sensorValue)) {
    gotAny = true;

    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        imuData.imuLinAccel[0] = sensorValue.un.linearAcceleration.x;
        imuData.imuLinAccel[1] = sensorValue.un.linearAcceleration.y;
        imuData.imuLinAccel[2] = sensorValue.un.linearAcceleration.z;
        gotAccel = true;
        break;

      case SH2_GAME_ROTATION_VECTOR:
        imuData.imuQuat[0] = sensorValue.un.gameRotationVector.i;
        imuData.imuQuat[1] = sensorValue.un.gameRotationVector.j;
        imuData.imuQuat[2] = sensorValue.un.gameRotationVector.k;
        imuData.imuQuat[3] = sensorValue.un.gameRotationVector.real;
        quatNormalize(imuData.imuQuat);
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        imuData.imuGyro[0] = sensorValue.un.gyroscope.x;
        imuData.imuGyro[1] = sensorValue.un.gyroscope.y;
        imuData.imuGyro[2] = sensorValue.un.gyroscope.z;
        break;

      default:
        break;
    }
  }

  if (!gotAny) return;

  if (!gotAccel) return;

  const uint32_t dtUs = (lastAccelRxUs == 0) ? 0 : (uint32_t)(nowUs - lastAccelRxUs);

  imuData.timestamp_us = (uint64_t)nowUs;
  imuData.dt_us = dtUs;
  lastAccelRxUs = nowUs;

  recomputeDerivedState();

  imuData.fresh = true;
}