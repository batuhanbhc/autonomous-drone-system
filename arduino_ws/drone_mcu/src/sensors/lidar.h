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
// Default baud rate: 115200, output rate: 100 Hz

#define LIDAR_BAUD      115200
#define LIDAR_FRAME_LEN 9
#define LIDAR_HEADER    0x59

struct LidarData {
    uint16_t distanceCm;   // distance in cm
    uint16_t strength;     // signal strength (arbitrary units)
    float    tempC;        // chip temperature in °C
    bool     fresh;
};

extern LidarData lidarData;

// Call once in setup(). Pass the HardwareSerial to use and its RX/TX pins.
bool lidarBegin(HardwareSerial &serial, int rxPin, int txPin);

// Call every loop(). Drains available bytes and sets lidarData.fresh when a
// valid frame is decoded.
void lidarUpdate();