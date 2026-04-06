#pragma once
#include <Arduino.h>

// TFS20-L UART frame format:
//   Byte 0:    0x59  (frame header 1)
//   Byte 1:    0x59  (frame header 2)
//   Byte 2:    Dist_L  (distance low byte, cm)
//   Byte 3:    Dist_H  (distance high byte, cm)
//   Byte 4:    Strength_L
//   Byte 5:    Strength_H
//   Byte 6:    Temp_L   (raw temperature low byte)
//   Byte 7:    Temp_H   (raw temperature high byte)
//   Byte 8:    Checksum (low byte of sum of bytes 0-7)
//
// Temperature (°C) = (Temp_H << 8 | Temp_L) / 8 - 256
#define LIDAR_BAUD        115200
#define LIDAR_FRAME_LEN   9
#define LIDAR_HEADER      0x59
#define LIDAR_RX_PIN      D7   // MCU RX  ← sensor TX (pin 3)
#define LIDAR_TX_PIN      D6   // MCU TX  → sensor RX (pin 4)

#define LIDAR_TRIGGER_HZ  20
#define LIDAR_TRIGGER_US  (1000000UL / LIDAR_TRIGGER_HZ)

#define LIDAR_MIN_RANGE_M   0.05f
#define LIDAR_MAX_RANGE_M   20.0f
#define LIDAR_MIN_RANGE_CM  5
#define LIDAR_MAX_RANGE_CM  2000
#define LIDAR_MIN_STRENGTH  200


// LiDAR position relative to drone center, in BODY frame.
// x = right, y = front, z = body-Z
static constexpr float lidarOffsetX_m = 0.00f;   // right +
static constexpr float lidarOffsetY_m = -0.072f;   // front +
static constexpr float lidarOffsetZ_m = 0.00f;   // body-Z +

struct LidarData {
    uint16_t distanceCm = -1;   // distance in cm
    uint16_t strength = 0;     // signal strength (arbitrary units)
    float    tempC = 0.0;        // chip temperature in °C
    uint32_t timestampMs = 0;
    bool     fresh = false;
};

extern LidarData lidarData;

// Call once in setup(). Configures trigger mode (frame rate = 0).
bool lidarBegin(HardwareSerial &serial);

// Send a single-measurement trigger command to the sensor.
void lidarTrigger();

// Call every loop(). Drains available bytes and sets lidarData.fresh when a
// valid frame is decoded.
void lidarUpdate();

bool lidarGetVerticalM(float*, 
    const float& qx, const float& qy, const float& qz, const float& qw,
    const uint16_t distanceCm, const uint16_t strength);  // tilt-corrected vertical distance in metres

float lidarGetAbsR22(const float& qx, const float& qy, const float& qz, const float& qw);