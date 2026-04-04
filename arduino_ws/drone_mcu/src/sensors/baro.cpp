#include "baro.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define BARO_ADDR       0x77
#define BARO_INT_PIN    19
#define SEA_LEVEL_HPA   1013.25f

// BMP390 register addresses
#define BMP3_REG_INT_CTRL   0x19
#define BMP3_REG_PWR_CTRL   0x1B

static Adafruit_BMP3XX bmp;
BaroData baroData = {};

static volatile bool baroIntFired = false;


// ── Low-level register helpers (I2C) ─────────────────────────────────────────
static void bmpWriteReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BARO_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static uint8_t bmpReadReg(uint8_t reg) {
    Wire.beginTransmission(BARO_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BARO_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ── ISR ──────────────────────────────────────────────────────────────────────
static void IRAM_ATTR baroISR() {
    baroIntFired = true;
}

// ── Helpers ──────────────────────────────────────────────────────────────────
static float pressureToAltitude(float pressurePa, float seaLevelHpa) {
    float atmospheric = pressurePa / 100.0f;
    return 44330.0f * (1.0f - powf(atmospheric / seaLevelHpa, 0.1903f));
}

// ── Public API ───────────────────────────────────────────────────────────────
bool baroBegin() {
    if (!bmp.begin_I2C(BARO_ADDR, &Wire)) {
        Serial.println("[BARO] FATAL: BMP390 not found on I2C!");
        return false;
    }

    // Sampling config — same as before
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // ── INT_CTRL (0x19) ───────────────────────────────────────────────────
    // Bit 6: drdy_en   = 1  → data-ready fires INT pin
    // Bit 2: int_latch = 0  → pulsed (not latched)
    // Bit 1: int_od    = 0  → push-pull
    // Bit 0: int_level = 1  → active HIGH
    bmpWriteReg(BMP3_REG_INT_CTRL, 0x42);  // 0b01000010

    // ── PWR_CTRL (0x1B) ───────────────────────────────────────────────────
    // Bit 5:4 mode     = 11 → normal mode (continuous)
    // Bit 1:  temp_en  = 1  → temperature sensor on
    // Bit 0:  press_en = 1  → pressure sensor on
    bmpWriteReg(BMP3_REG_PWR_CTRL, 0x33);  // 0b00110011

    // Verify mode was accepted
    uint8_t pwr = bmpReadReg(BMP3_REG_PWR_CTRL);
    if ((pwr & 0x30) != 0x30) {
        Serial.printf("[BARO] WARN: PWR_CTRL readback=0x%02X, expected 0x33\n", pwr);
    }

    // ── GPIO interrupt ────────────────────────────────────────────────────
    pinMode(BARO_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BARO_INT_PIN), baroISR, RISING);

    Serial.println("[BARO] BMP390 ready — normal mode 50Hz, INT→GPIO19");
    return true;
}

void baroUpdate() {
    if (!baroIntFired) return;
    baroIntFired = false;

    if (!bmp.performReading()) return;

    // ── Store results ─────────────────────────────────────────────────────
    baroData.pressurePa = bmp.pressure;
    baroData.tempC      = bmp.temperature;
    baroData.altitudeM  = pressureToAltitude(baroData.pressurePa, SEA_LEVEL_HPA);
    baroData.fresh      = true;
}

float baroGetRelativeAltitudeM() {
    if (baroData.launchPressurePa <= 1000.0f || baroData.pressurePa <= 1000.0f) {
        return 0.0f;
    }

    const float p     = baroData.pressurePa / 100.0f;
    const float p0    = baroData.launchPressurePa / 100.0f;
    return 44330.0f * (1.0f - powf(p / p0, 0.1903f));
}