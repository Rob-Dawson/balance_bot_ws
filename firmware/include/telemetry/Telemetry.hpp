#pragma once
#include <stdint.h> //Uncomment if using AVR-GCC: 7.3.0 and older
// #include <cstdint>  //Uncomment if using ARM
struct Telemetry {
    float rawPitch;
    float pitchError;
    float targetPitch;
    float pitchEstimate;
    float pitchRateEstimate;

    float outputKp;
    float outputKd;
    float outputKi;
    float commandOutput;

    float requestedPWMLeft;
    float requestedPWMRight;
    int16_t appliedPWMLeft;
    int16_t appliedPWMRight;
    float speedRight;
    float speedLeft;
};
