#include "BalanceBot.hpp"
#include <Arduino.h>

BalanceBot robot;
void setup() {
    Serial.begin(230400);
    robot.init();
}

void loop() { robot.update(); }
