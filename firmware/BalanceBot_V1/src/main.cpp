#include <Arduino.h>
#include "IMUStateMachine.hpp"
#include "Encoder.hpp"
#include "MotorDriver.hpp"
#include "BalanceController.hpp"
const char* stateToString(IMUState state)
{
    switch (state)
    {
        case IMUState::CALIBRATING_GYRO:
            return "CALIBRATING_GYRO";
        case IMUState::CALIBRATING_ZERO:
            return "CALIBRATING_ZERO";
        case IMUState::RUNNING:
            return "RUNNING";
        default:
            return "UNKNOWN";
    }
}

IMUStateMachine imu;
IMUState previousState = imu.getState();

Encoder encoder;
MotorDriver motor;
BalanceController controller;
float controlOutput = 0.0f;
void setup() {
  Serial.begin(9600);
  imu.imuInit();
  encoder.init();  
  motor.init();
}

void onEnterState(IMUState state)
{
  Serial.println(stateToString(state));
}

void loop() {
  imu.update();
  IMUState state = imu.getState();
  if (state != previousState)
  {
    onEnterState(state);
    previousState = state;
  }
  if (state != IMUState::RUNNING) return; 
  controlOutput = controller.update(imu.getPitch(), imu.getPitchRate());
  motor.setLeftPWM(controlOutput);
  motor.setRightPWM(controlOutput);

}
