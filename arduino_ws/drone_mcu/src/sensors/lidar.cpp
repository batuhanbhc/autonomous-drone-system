#include "lidar.h"
#include "imu.h"


static HardwareSerial* lidarSerial = nullptr;
LidarData lidarData = {0, 0, 0.0f, false};

// Set output frame rate.
// rate == 0  →  trigger mode (sensor only outputs when triggered)
// Cmd 0x03: 5A 06 03 RL RH SU
static void sendFrameRate(uint16_t rate) {
    uint8_t rl = rate & 0xFF;
    uint8_t rh = (rate >> 8) & 0xFF;
    uint8_t su = (0x5A + 0x06 + 0x03 + rl + rh) & 0xFF;
    const uint8_t cmd[] = {0x5A, 0x06, 0x03, rl, rh, su};
    lidarSerial->write(cmd, sizeof(cmd));
    lidarSerial->flush();
}

bool lidarBegin(HardwareSerial& port) {
    lidarSerial = &port;
    port.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    delay(50);

    // Enable data output (required on every power-on for TFS20-L)
    const uint8_t enableOutput[] = {0x5A, 0x05, 0x07, 0x01, 0x67};
    port.write(enableOutput, sizeof(enableOutput));
    port.flush();
    delay(20);

    // Switch to trigger mode: frame rate = 0
    sendFrameRate(0);
    delay(20);

    return true;
}

// Single-measurement trigger.
// Cmd 0x04: 5A 04 04 62
void lidarTrigger() {
    if (!lidarSerial) return;
    const uint8_t cmd[] = {0x5A, 0x04, 0x04, 0x62};
    lidarSerial->write(cmd, sizeof(cmd));
}

void lidarUpdate() {
    if (!lidarSerial) return;

    // ── Sensor reading trigger timer ────────────────────────────────────────────────
    static uint32_t nextTriggerUs = 0;
    uint32_t nowUs = micros();

    if (nextTriggerUs == 0) {
        nextTriggerUs = nowUs;   // initialize once
    }

    if ((int32_t)(nowUs - nextTriggerUs) >= 0) {
        nextTriggerUs += LIDAR_TRIGGER_US;   // phase-locked schedule
        lidarTrigger();
    }

    // ── Frame parser ──────────────────────────────────────────────────────
    static uint8_t frame[LIDAR_FRAME_LEN];
    static uint8_t idx = 0;

    while (lidarSerial->available()) {
        uint8_t b = lidarSerial->read();

        if (idx == 0 && b != LIDAR_HEADER) continue;
        if (idx == 1 && b != LIDAR_HEADER) { idx = 0; continue; }

        frame[idx++] = b;

        if (idx == LIDAR_FRAME_LEN) {
            idx = 0;

            uint16_t sum = 0;
            for (int i = 0; i < 8; i++) sum += frame[i];
            if ((sum & 0xFF) != frame[8]) continue;   // bad checksum

            lidarData.timestampMs = millis();
            lidarData.distanceCm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
            lidarData.strength   = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
            uint16_t rawTemp     = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
            lidarData.tempC      = rawTemp / 8.0f - 256.0f;
            lidarData.fresh      = true;
        }
    }
}

bool lidarGetVerticalM(float* out_m,
    const float& qx, const float& qy, const float& qz, const float& qw,
    const uint16_t distanceCm, const uint16_t strength) {
    if (!out_m) return false;

    if (distanceCm < LIDAR_MIN_RANGE_CM || distanceCm > LIDAR_MAX_RANGE_CM || strength < LIDAR_MIN_STRENGTH) {
        return false;
    }

    const float R20 = 2.0f * (qx * qz - qy * qw);
    const float R21 = 2.0f * (qy * qz + qx * qw);
    const float R22 = 1.0f - 2.0f * (qx * qx + qy * qy);

    const float raw_m = 0.01f * distanceCm;
    const float hLidar = raw_m * fabsf(R22);

    const float sensorWorldZ =
        R20 * lidarOffsetX_m +
        R21 * lidarOffsetY_m +
        R22 * lidarOffsetZ_m;

    *out_m = hLidar - sensorWorldZ;
    return true;
}

float lidarGetAbsR22(const float& qx, const float& qy, const float& qz, const float& qw) {
    const float R22 = 1.0f - 2.0f * (qx * qx + qy * qy);
    return fabsf(R22);
}