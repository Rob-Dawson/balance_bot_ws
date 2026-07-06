#pragma once

#include "Encoder.hpp"
#include "IMUStateMachine.hpp"
#include "MotorDriver.hpp"
#include "PitchController.hpp"
#include "Telemetry.hpp"
#include "VelocityController.hpp"
#include <Arduino.h>
#include <stdint.h>

class BalanceBot {
public:
    void init();
    void update();

private:
    void closedLoop(float motorA, float motorB);
    void openLoop(int motorA, int motorB);

    void handleSerial();
    void telemetryPrint();
    void telemetryState();

    void onEnterState(IMUState state);

    IMUStateMachine imu;
    IMUState previousState{imu.getState()};

    Encoder encoder;
    MotorDriver motor;
    PitchController pitchController;
    VelocityController velocityController;
    Telemetry telemetry;

    float controlOutput{0.0f};

    uint32_t lastPrintTime{0};
    uint32_t m_lastUpdateTime{0};
    bool initPrint{true};

    float dtElapsed{0.0f};
    float m_desiredSpeed{0.0};
    const float m_wheelRad{0.35};
    const uint8_t printTimeThreshold{50};
};