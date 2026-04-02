#include "imu.h"

static Adafruit_BNO08x   bno08x(IMU_RESET_PIN);
static sh2_SensorValue_t sensorValue;

ImuData imuData = {};

static volatile bool imuInterruptFired = false;


static void quatToEuler(const float q[4], float out[3]) {
  // q = [i, j, k, real]  →  [x, y, z, w]
  float x = q[0], y = q[1], z = q[2], w = q[3];

  // Roll (X-axis rotation)
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  out[2] = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

  // Pitch (Y-axis rotation) — clamped to avoid NaN at poles
  float sinp = 2.0f * (w * y - z * x);
  sinp = sinp >  1.0f ?  1.0f : sinp;
  sinp = sinp < -1.0f ? -1.0f : sinp;
  out[1] = asinf(sinp) * RAD_TO_DEG;

  // Yaw (Z-axis rotation)
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  out[0] = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

static void gravToEuler(float x, float y, float z, float out[2]) {
  float norm = sqrtf(x*x + y*y + z*z);
  if (norm < 0.001f) { out[0] = out[1] = 0.0f; return; }
  x /= norm; y /= norm; z /= norm;
  out[0] = asinf(-x)         * RAD_TO_DEG;  // pitch
  out[1] = atan2f(y, z)      * RAD_TO_DEG;  // roll
}

void IRAM_ATTR imuISR() {
    imuInterruptFired = true;  // back to just this, nothing else
}

static bool setReports() {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION,  IMU_REPORT_RATE_US)) return false;
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_RATE_US)) return false;
  if (!bno08x.enableReport(SH2_GRAVITY, IMU_REPORT_RATE_US)) return false;
  return true;
}

bool imuBegin() {
  delay(200);
  Wire.begin(D4, D5);

  if (!bno08x.begin_I2C(IMU_ADDR, &Wire)) {
    Serial.println("[IMU] FATAL: Not detected on I2C!");
    return false;
  }

  Wire.setClock(400000);   // move here

  if (!setReports()) {
    Serial.println("[IMU] FATAL: Failed to set reports!");
    return false;
  }

  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuISR, FALLING);

  Serial.println("[IMU] Ready @ 200Hz with interrupt.");
  return true;
}

void imuUpdate() {
  if (bno08x.wasReset()) {
    Serial.println("[IMU] Reset detected, re-enabling reports...");
    setReports();
  }

  if (!imuInterruptFired) return;
  imuInterruptFired = false;


  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION: {
          imuData.linAccel[0] = sensorValue.un.linearAcceleration.x;
          imuData.linAccel[1] = sensorValue.un.linearAcceleration.y;
          imuData.linAccel[2] = sensorValue.un.linearAcceleration.z;

          imuData.fresh = true;
          break;
      }
      case SH2_GAME_ROTATION_VECTOR:
          imuData.quat[0] = sensorValue.un.gameRotationVector.i;
          imuData.quat[1] = sensorValue.un.gameRotationVector.j;
          imuData.quat[2] = sensorValue.un.gameRotationVector.k;
          imuData.quat[3] = sensorValue.un.gameRotationVector.real;
          quatToEuler(imuData.quat, imuData.euler);
          break;

      case SH2_GRAVITY:
          gravToEuler(sensorValue.un.gravity.x,
                      sensorValue.un.gravity.y,
                      sensorValue.un.gravity.z,
                      imuData.gravEuler);
          break;
    }
  }
}