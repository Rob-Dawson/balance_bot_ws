#include "Encoder.hpp"
#include "IMUStateMachine.hpp"
#include "MotorDriver.hpp"
#include "PitchController.hpp"
#include "Telemetry.hpp"
#include "VelocityController.hpp"
#include <Arduino.h>

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
    inline float convertToDeg(float rad) { return rad * 180 / PI; }
    void onEnterState(IMUState state);

    static const char *stateToString(IMUState state);

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

    static constexpr float microToSeconds{0.000001f};
    float dtElapsed{0.0f};
    static constexpr float m_wheelRad{0.35};
    float m_desiredSpeed{0.0};
};