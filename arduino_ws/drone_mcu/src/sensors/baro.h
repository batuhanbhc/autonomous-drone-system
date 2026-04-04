#pragma once
#include <Arduino.h>

struct BaroData {
    float pressurePa = 0.0f;
    float launchPressurePa = 0.0f;
    float tempC = 0.0f;
    float altitudeM = 0.0f;
    bool  fresh = false;
};

extern BaroData baroData;

bool baroBegin();
void baroUpdate();
float baroGetRelativeAltitudeM();