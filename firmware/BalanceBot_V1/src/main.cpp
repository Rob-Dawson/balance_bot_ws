#include "balance/BalanceBot.hpp"
#include "encoder/Encoder.hpp"
#include "imu/IMUStateMachine.hpp"
#include <Arduino.h>

ArduinoEncoder encoder;
IMUStateMachine imu;

// BalanceBot robot{imu};
BalanceBot robot{imu, &encoder};

constexpr uint32_t baudrate = 230400;
void setup() {
    Serial.begin(baudrate);
    Wire.begin();
    if (!robot.init()) {
        Serial.println("Robot initialisation failed");
        Serial.println(BalanceBot::componentName(BalanceBot::IMU));
        Serial.println(BalanceBot::componentState(
            robot.getComponentState(BalanceBot::IMU)));
    }
    while (true) {
    }
}

void loop() { robot.update(); }
