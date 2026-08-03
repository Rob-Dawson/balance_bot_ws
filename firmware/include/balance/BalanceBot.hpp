#pragma once
#include "controller/PitchController.hpp"
#include "controller/VelocityController.hpp"

#include "encoder/Encoder.hpp"
#include "imu/IIMU.hpp"
#include "motor/MotorDriver.hpp"
#include "telemetry/Telemetry.hpp"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

class BalanceBot {
public:
    enum Component { IMU, ENCODER, MOTOR, TELEMETRY, COUNT };
    enum ComponentStatus {
        NOT_PRESENT,
        NOT_INITIALISED,
        READY,
        INIT_FAILED,
    };
    bool init();
    void update();
    ComponentStatus getComponentState(Component component) const {
        return systemStates[component];
    };
    static const char *componentName(Component component) {
        switch (component) {
        case IMU:
            return "IMU";
        case ENCODER:
            return "Encoder";
        case MOTOR:
            return "Motor";
        case TELEMETRY:
            return "Telemetry";
        default:
            return "UNKNOWN";
        }
    }

    static const char *componentState(ComponentStatus componentState) {
        switch (componentState) {
        case NOT_PRESENT:
            return "NOT_PRESENT";
        case NOT_INITIALISED:
            return "NOT_INITIALISED";
        case READY:
            return "READY";
        case INIT_FAILED:
            return "INIT_FAILED";
        default:
            return "UNKNOWN";
        }
    }

    BalanceBot(IIMU &imu, ArduinoEncoder *encoder = nullptr,
               Telemetry *telemetry = nullptr);

private:
    ComponentStatus systemStates[COUNT];

    template <typename ComponentType>
    void initHelper(ComponentType &component, Component componentID) {
        if (component.init()) {
            systemStates[componentID] = ComponentStatus::READY;
        } else {
            systemStates[componentID] = ComponentStatus::INIT_FAILED;
        }
    }

    template <typename ComponentType>
    void initHelperOptional(ComponentType *component, Component componentID) {
        if (component == nullptr) {
            systemStates[componentID] = ComponentStatus::NOT_PRESENT;
            return;
        }
        initHelper(*component, componentID);
    }

    void closedLoop(float motorA, float motorB);
    void openLoop(int motorA, int motorB);

    void handleSerial();
    void telemetryPrint();
    void telemetryState();

    void onEnterState(IMUState state);

    IIMU &imu;
    IMUState previousState{imu.getState()};

    ArduinoEncoder *encoder;
    Telemetry *telemetry;
    MotorDriver motor;
    PitchController pitchController;
    VelocityController velocityController;

    float controlOutput{0.0f};

    uint32_t lastPrintTime{0};
    uint32_t m_lastUpdateTime{0};
    bool initPrint{true};

    float dtElapsed{0.0f};
    float m_desiredSpeed{0.0};
    const float m_wheelRad{0.35};
    const uint8_t printTimeThreshold{50};

    bool canBalance() {
        return systemStates[IMU] == ComponentStatus::READY &&
               systemStates[MOTOR] == ComponentStatus::READY;
    };
    bool canControlVelocity() {
        return systemStates[ENCODER] == ComponentStatus::READY;
    };
    bool hasTelemetry() {
        return systemStates[TELEMETRY] == ComponentStatus::READY;
    }
    void updateIMU();
    void updateTelemetry();
    float updateVelocityControl();
};
