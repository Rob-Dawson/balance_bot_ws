#include <Arduino.h>
#include "IMUStateMachine.hpp"
#include "Encoder.hpp"
#include "MotorDriver.hpp"

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

  motor.setRightPWM(0);
  motor.setLeftPWM(0);

  Serial.print("Left Wheel Speed: ");
  Serial.print(encoder.getSpeedLeft(),2);

  Serial.print("\t\t\tRight Wheel Speed: ");
  Serial.println(encoder.getSpeedRight(),2);

}
