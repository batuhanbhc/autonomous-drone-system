#include <Arduino.h>
#include "src/sensors/imu.h"
#include "src/sensors/baro.h"
#include "src/sensors/lidar.h"

// TFS20-L wiring (UART mode — INT pin tied to GND at power-on):
//   Pin 1  → 3V3
//   Pin 2  → GND  (3V3GND)
//   Pin 3  → UART_TX of sensor → RX pin of MCU  (LIDAR_RX_PIN)
//   Pin 4  → UART_RX of sensor → TX pin of MCU  (LIDAR_TX_PIN)
//   Pin 5  → GND  (INT tied low to select UART)
//   Pin 6  → GND
#define LIDAR_RX_PIN  D7   // MCU RX  ← sensor TX (pin 3)
#define LIDAR_TX_PIN  D6   // MCU TX  → sensor RX (pin 4)  (unused but required by begin())

void setup() {
    Serial.begin(460800);
    delay(1000);

    if (!imuBegin())              { while (1) delay(500); }
    if (!baroBegin())             { while (1) delay(500); }
    if (!lidarBegin(Serial1, LIDAR_RX_PIN, LIDAR_TX_PIN)) { while (1) delay(500); }
}

static uint32_t decim = 0;

void loop() {
    imuUpdate();
    baroUpdate();
    lidarUpdate();

    if (imuData.fresh) {
        imuData.fresh  = false;
        baroData.fresh = false;   // consume together
        decim++;
        if (decim >= 40) {        // print at ~5 Hz (200 Hz IMU / 40)
            decim = 0;

            // Snapshot lidar (may be stale if sensor slower than IMU)
            bool  lidarOk = lidarData.distanceCm > 0 && lidarData.strength > 100;
            float distM   = lidarData.distanceCm / 100.0f;

            Serial.printf(
                "LinAccel X=%.4f Y=%.4f Z=%.4f | "
                "Quat  Yaw=%.2f Pit=%.2f Rol=%.2f | "
                "Grav        Pit=%.2f Rol=%.2f | "
                "dPit=%.3f dRol=%.3f | "
                "Press=%.2fhPa Temp=%.2fC Alt=%.2fm | "
                "Lidar %s=%.3fm Str=%u LidarT=%.1fC\r\n",
                imuData.linAccel[0], imuData.linAccel[1], imuData.linAccel[2],
                imuData.euler[0],    imuData.euler[1],    imuData.euler[2],
                imuData.gravEuler[0], imuData.gravEuler[1],
                imuData.euler[1] - imuData.gravEuler[0],   // Δpitch
                imuData.euler[2] - imuData.gravEuler[1],   // Δroll
                baroData.pressurePa / 100.0f, baroData.tempC, baroData.altitudeM,
                lidarOk ? "D" : "!",
                distM, lidarData.strength, lidarData.tempC);
        }
    }
}