#pragma once
#include <Arduino.h>

struct BaroData {
    float pressurePa;
    float tempC;
    float altitudeM;
    bool  fresh;
};

extern BaroData baroData;

bool baroBegin();
void baroUpdate();