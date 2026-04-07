#include <Arduino.h>
#include "src/sensors/imu.h"
#include "src/sensors/baro.h"
#include "src/sensors/lidar.h"
#include "src/estimation/altitude_ekf.h"

static uint32_t decim = 0;

static constexpr uint32_t INIT_WARMUP_MS   = 1500;
static constexpr uint32_t INIT_DURATION_MS = 5000;

// =========================
// Companion packet settings
// =========================
static constexpr uint8_t  COMPANION_SYNC0        = 0xA5;
static constexpr uint8_t  COMPANION_SYNC1        = 0x5A;
static constexpr uint8_t  COMPANION_MSG_VERTICAL = 0x01;
static constexpr uint32_t COMPANION_SEND_HZ      = 30;
static constexpr uint32_t COMPANION_SEND_PERIOD_MS = 1000UL / COMPANION_SEND_HZ;

// Set to 1 if you want to stop text prints after boot and only emit binary.
static constexpr bool USB_BINARY_ONLY_AFTER_INIT = true;

// Rolling sequence number so the Pi can detect packet loss/reordering.
static uint8_t companionSeq = 0;
static uint32_t nextCompanionSendMs = 0;

// Packed payload that the Pi will decode.
#pragma pack(push, 1)
struct VerticalEstimatePayload {
  uint32_t timestamp_ms;
  float z_world_m;
  float vz_world_mps;
  float agl_m;
  uint8_t ekf_initialized;
  uint8_t lidar_accepted;
};
#pragma pack(pop)

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

// =========================
// CRC16-CCITT-FALSE
// poly = 0x1021, init = 0xFFFF
// =========================
static uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// =========================
// Companion packet sender
// Frame:
//   [0]  SYNC0
//   [1]  SYNC1
//   [2]  MSG_ID
//   [3]  PAYLOAD_LEN
//   [4]  SEQ
//   [5..] payload
//   [N]  CRC low
//   [N+1] CRC high
//
// CRC is computed over:
//   MSG_ID, PAYLOAD_LEN, SEQ, PAYLOAD...
// =========================
static void sendVerticalEstimatePacket() {
  VerticalEstimatePayload p{};
  p.timestamp_ms       = millis();
  p.z_world_m          = altitudeEkf.s.z_m;
  p.vz_world_mps       = altitudeEkf.s.vz_mps;
  p.agl_m              = altitudeEkf.s.agl_m;
  p.ekf_initialized    = altitudeEkf.s.initialized ? 1 : 0;
  p.lidar_accepted     = altitudeEkf.s.lastLidarAccepted ? 1 : 0;

  const uint8_t payloadLen = (uint8_t)sizeof(VerticalEstimatePayload);

  uint8_t crcBuf[3 + sizeof(VerticalEstimatePayload)];
  crcBuf[0] = COMPANION_MSG_VERTICAL;
  crcBuf[1] = payloadLen;
  crcBuf[2] = companionSeq;
  memcpy(&crcBuf[3], &p, sizeof(p));

  const uint16_t crc = crc16CcittFalse(crcBuf, sizeof(crcBuf));

  Serial.write(COMPANION_SYNC0);
  Serial.write(COMPANION_SYNC1);
  Serial.write(COMPANION_MSG_VERTICAL);
  Serial.write(payloadLen);
  Serial.write(companionSeq);
  Serial.write((const uint8_t*)&p, sizeof(p));
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)((crc >> 8) & 0xFF));

  companionSeq++;
}

static void maybeSendCompanionPacket() {
  const uint32_t nowMs = millis();

  if (nextCompanionSendMs == 0) {
    nextCompanionSendMs = nowMs;
  }

  if ((int32_t)(nowMs - nextCompanionSendMs) >= 0) {
    nextCompanionSendMs += COMPANION_SEND_PERIOD_MS;  // phase-locked 20 Hz
    sendVerticalEstimatePacket();
  }
}

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
  imuComputeLevelCorrection();
  
  const float baroRel0 = 0.0f;
  const bool lidarOk = lidarData.fresh &&
                     lidarData.distanceCm >= LIDAR_MIN_RANGE_CM &&
                     lidarData.distanceCm <= LIDAR_MAX_RANGE_CM &&
                     lidarData.strength   >= LIDAR_MIN_STRENGTH;
  float lidar0 = lidarOk ? lidarData.distanceCm * 0.01f : 0.0f;

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

  if (USB_BINARY_ONLY_AFTER_INIT) {
    Serial.println("[COMPANION] Starting binary vertical packets at 20 Hz");
    delay(20);
  }

  nextCompanionSendMs = millis();
  return true;
}

static bool imuOk   = false;
static bool baroOk  = false;
static bool lidarOk = false;

void setup() {
  Serial.begin(460800);
  delay(1000);

  uint8_t attempt_count = 0;
  Serial.println("[BOOT] STARTING IMU");
  do {
    imuOk = imuBegin();
    attempt_count++;
    if (!imuOk && attempt_count < 10) delay(500);
  } while (!imuOk && attempt_count < 10);
  Serial.printf("[BOOT] IMU   : %s\r\n", imuOk ? "OK" : "FAILED");

  attempt_count = 0;
  Serial.println("[BOOT] STARTING BARO");
  do {
    baroOk = baroBegin();
    attempt_count++;
    if (!baroOk && attempt_count < 10) delay(500);
  } while (!baroOk && attempt_count < 10);
  Serial.printf("[BOOT] BARO  : %s\r\n", baroOk ? "OK" : "FAILED");

  attempt_count = 0;
  Serial.println("[BOOT] STARTING LIDAR");
  do {
    lidarOk = lidarBegin(Serial1);
    attempt_count++;
    if (!lidarOk && attempt_count < 10) delay(500);
  } while (!lidarOk && attempt_count < 10);
  Serial.printf("[BOOT] LIDAR : %s\r\n", lidarOk ? "OK" : "FAILED");

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

    const float qx = imuData.droneQuat[0];
    const float qy = imuData.droneQuat[1];
    const float qz = imuData.droneQuat[2];
    const float qw = imuData.droneQuat[3];
    const uint16_t dist_cm = lidarData.distanceCm;
    const uint16_t strength = lidarData.strength;

    float lidarVertical_m = 0.0f;
    if (lidarGetVerticalM(&lidarVertical_m, qx, qy, qz, qw, dist_cm, strength)) {
        const float absR22 = lidarGetAbsR22(qx, qy, qz, qw);

        altitudeEkfUpdateLidar(
          lidarVertical_m,
          dist_cm,
          lidarData.strength,
          absR22,
          millis()
        );
    };
  }

  // 4) Send binary packet
  maybeSendCompanionPacket();
}