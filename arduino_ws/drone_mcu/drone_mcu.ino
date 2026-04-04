#include <Arduino.h>
#include "src/sensors/imu.h"
#include "src/sensors/baro.h"
#include "src/sensors/lidar.h"
#include "src/estimation/altitude_ekf.h"

static uint32_t decim = 0;

static constexpr uint32_t INIT_WARMUP_MS   = 1500;
static constexpr uint32_t INIT_DURATION_MS = 3000;

struct InitAccum {
  bool running = false;
  bool done    = false;

  uint32_t startMs = 0;

  uint32_t imuSamples  = 0;
  uint32_t baroSamples = 0;
  uint32_t gyroSamples = 0;

  double accSum[3]    = {0.0, 0.0, 0.0};
  double gyroSum[3]   = {0.0, 0.0, 0.0};
  double pressureSum  = 0.0;
};

static InitAccum initAccum;

static void beginInitializationRoutine() {
  imuData.initialized = false;

  imuData.bias.droneLinAccel[0] = 0.0f;
  imuData.bias.droneLinAccel[1] = 0.0f;
  imuData.bias.droneLinAccel[2] = 0.0f;
  imuData.bias.droneGyro[0]     = 0.0f;
  imuData.bias.droneGyro[1]     = 0.0f;
  imuData.bias.droneGyro[2]     = 0.0f;

  initAccum = {};
  initAccum.running = true;
  initAccum.startMs = millis();

  Serial.println("[INIT] Collecting stationary samples...");
}

static bool updateInitializationRoutine() {
  if (!initAccum.running || initAccum.done) return initAccum.done;

  // collect IMU-derived data
  if (imuData.fresh) {
    imuData.fresh = false;

    initAccum.accSum[0] += imuData.droneLinAccel[0];
    initAccum.accSum[1] += imuData.droneLinAccel[1];
    initAccum.accSum[2] += imuData.droneLinAccel[2];

    initAccum.gyroSum[0] += imuData.droneGyro[0];
    initAccum.gyroSum[1] += imuData.droneGyro[1];
    initAccum.gyroSum[2] += imuData.droneGyro[2];

    initAccum.imuSamples++;
    initAccum.gyroSamples++;
  }

  // collect baro pressure
  if (baroData.fresh) {
    baroData.fresh = false;

    initAccum.pressureSum += baroData.pressurePa;
    initAccum.baroSamples++;
  }

  if ((millis() - initAccum.startMs) < INIT_DURATION_MS) {
    return false;
  }

  initAccum.running = false;
  initAccum.done    = true;

  if (initAccum.imuSamples > 0) {
    imuData.bias.droneLinAccel[0] = initAccum.accSum[0] / initAccum.imuSamples;
    imuData.bias.droneLinAccel[1] = initAccum.accSum[1] / initAccum.imuSamples;
    imuData.bias.droneLinAccel[2] = initAccum.accSum[2] / initAccum.imuSamples;
  }

  if (initAccum.gyroSamples > 0) {
    imuData.bias.droneGyro[0] = initAccum.gyroSum[0] / initAccum.gyroSamples;
    imuData.bias.droneGyro[1] = initAccum.gyroSum[1] / initAccum.gyroSamples;
    imuData.bias.droneGyro[2] = initAccum.gyroSum[2] / initAccum.gyroSamples;
  }

  if (initAccum.baroSamples > 0) {
    baroData.launchPressurePa = initAccum.pressureSum / initAccum.baroSamples;
  }

  imuData.initialized = true;

  const float baroRel0 = 0.0f;  // launch-relative by definition
  float lidar0 = 0.0f;
  const bool lidarOk = lidarGetVerticalM(&lidar0);

  altitudeEkfInitialize(baroRel0, lidarOk, lidar0);

  Serial.printf("\tNum samples: accel=%lu gyro=%lu baro=%lu\r\n",
    (unsigned long)initAccum.imuSamples,
    (unsigned long)initAccum.gyroSamples,
    (unsigned long)initAccum.baroSamples);

  Serial.printf("\tAccel biases: x=%.6f y=%.6f z=%.6f\r\n",
    imuData.bias.droneLinAccel[0],
    imuData.bias.droneLinAccel[1],
    imuData.bias.droneLinAccel[2]);

  Serial.printf("\tGyro biases: x=%.6f y=%.6f z=%.6f\r\n",
    imuData.bias.droneGyro[0],
    imuData.bias.droneGyro[1],
    imuData.bias.droneGyro[2]);


  Serial.printf("\tLaunch Pressure (Pa): %.2f\r\n",
    (double)baroData.launchPressurePa);

  return true;
}


static bool imuOk   = false;
static bool baroOk  = false;
static bool lidarOk = false;

void setup() {
  Serial.begin(460800);
  delay(1000);

  // ---- STARTING IMU ----
  uint8_t attempt_count = 0;
  Serial.println("[BOOT] STARTING IMU");
  do {
    imuOk = imuBegin();
    attempt_count++;
    if (!imuOk && attempt_count < 3) delay(500);
  } while (!imuOk && attempt_count < 3);
  Serial.printf("[BOOT] IMU   : %s\r\n", imuOk ? "OK" : "FAILED");
  // -----------------------

  // ---- STARTING BARO ----
  attempt_count = 0;
  Serial.println("[BOOT] STARTING BARO");
  do {
    baroOk = baroBegin();
    attempt_count++;
    if (!baroOk && attempt_count < 3) delay(500);
  } while (!baroOk && attempt_count < 3);
  Serial.printf("[BOOT] BARO  : %s\r\n", baroOk ? "OK" : "FAILED");
  // -----------------------

  // ---- STARTING LIDAR ----
  attempt_count = 0;
  Serial.println("[BOOT] STARTING LIDAR");
  do {
    lidarOk = lidarBegin(Serial1);
    attempt_count++;
    if (!lidarOk && attempt_count < 3) delay(500);
  } while (!lidarOk && attempt_count < 3);
  Serial.printf("[BOOT] LIDAR : %s\r\n", lidarOk ? "OK" : "FAILED");
  // -----------------------

  if (!imuOk && !baroOk && !lidarOk) {
    Serial.println("[BOOT] No sensors could be started.");
    while (1) delay(500);
  }

  Serial.println("[BOOT] Sensor open attempts finished, warming up...");
  delay(INIT_WARMUP_MS);

  beginInitializationRoutine();
}


void loop() {
  imuUpdate();
  baroUpdate();
  lidarUpdate();

  if (!imuData.initialized) {
    updateInitializationRoutine();
    return;
  }

  // 1) Predict on IMU
  if (imuData.fresh) {
    imuData.fresh = false;

    const float dt_s = imuData.dt_us * 1e-6f;
    altitudeEkfPredict(imuData.worldLinAccelZ, dt_s);
  }

  // 2) Update on baro
  if (baroData.fresh) {
    baroData.fresh = false;

    const float zBaroRel_m = baroGetRelativeAltitudeM();
    altitudeEkfUpdateBaro(zBaroRel_m);
  }

  // 3) Update on lidar
  if (lidarData.fresh) {
    lidarData.fresh = false;

    float lidarVertical_m = 0.0f; 
    lidarGetVerticalM(&lidarVertical_m);
    const float absR22 = lidarGetAbsR22();

    altitudeEkfUpdateLidar(
      lidarVertical_m,
      lidarData.strength,
      absR22,
      millis()
    );
  }

  // 4) Decimated debug
  decim++;
  if (decim >= 50) {
    decim = 0;

    Serial.printf(
        "AGL=%.2f vz=%.2f gc=%.2f\r\n",
        altitudeEkf.s.agl_m,
        altitudeEkf.s.vz_mps,
        altitudeEkf.s.groundConfidence
    );
  }
}