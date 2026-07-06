#include "BalanceBot.hpp"
#include <Arduino.h>

BalanceBot robot;
constexpr uint32_t baudrate = 230400;
void setup() {
    Serial.begin(baudrate);
    robot.init();
}

void loop() { robot.update(); }
