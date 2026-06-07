#include <Arduino.h>
#include "IMUStateMachine.hpp"
IMUStateMachine imu;

void setup() {
  Serial.begin(9600);
  imu.imuInit();
}

void loop() {
  imu.update();
}
