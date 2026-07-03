#include "BalanceController.hpp"

BalanceController::BalanceController() {
  m_pitchController.setOutputLimits(-255, 255);
  m_pitchController.setPD(1000.0, 150.0);
}

void BalanceController::setKp(float Kp) { m_pitchController.setP(Kp); }

void BalanceController::setKd(float Kd) { m_pitchController.setD(Kd); }

float BalanceController::update(float pitch, float pitchRate) {
  m_error = m_targetPitch - pitch;
  float controlOutput = m_pitchController.updatePD(m_error, pitchRate);
  return controlOutput;
}
