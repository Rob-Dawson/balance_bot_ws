#include "BalanceController.hpp"
#include "Encoder.hpp"
#include "IMUStateMachine.hpp"
#include "MotorDriver.hpp"
#include "Telemetry.hpp"
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

    const char *stateToString(IMUState state);

    IMUStateMachine imu;
    IMUState previousState = imu.getState();

    Encoder encoder;
    MotorDriver motor;
    BalanceController controller;
    Telemetry telemetry;

    float controlOutput = 0.0f;

    uint32_t lastPrintTime = 0;
    bool initPrint = true;
};