#include "lidar.h"

static HardwareSerial* lidarSerial = nullptr;

LidarData lidarData = {0, 0, 0.0f, false};

bool lidarBegin(HardwareSerial& port, int rxPin, int txPin) {
    lidarSerial = &port;

    port.begin(115200, SERIAL_8N1, rxPin, txPin);
    delay(50);

    const uint8_t enableOutput[] = {0x5A, 0x05, 0x07, 0x01, 0x67};
    port.write(enableOutput, sizeof(enableOutput));
    port.flush();
    delay(20);

    return true;
}

void lidarUpdate() {
    if (!lidarSerial) return;

    static uint8_t frame[LIDAR_FRAME_LEN];
    static uint8_t idx = 0;

    while (lidarSerial->available()) {
        uint8_t b = lidarSerial->read();

        if (idx == 0 && b != LIDAR_HEADER) continue;
        if (idx == 1 && b != LIDAR_HEADER) {
            idx = 0;
            continue;
        }

        frame[idx++] = b;

        if (idx == LIDAR_FRAME_LEN) {
            idx = 0;

            uint16_t sum = 0;
            for (int i = 0; i < 8; i++) sum += frame[i];
            if ((sum & 0xFF) != frame[8]) continue;

            lidarData.distanceCm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
            lidarData.strength   = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

            uint16_t rawTemp = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
            lidarData.tempC  = rawTemp / 8.0f - 256.0f;
            lidarData.fresh  = true;
        }
    }
}