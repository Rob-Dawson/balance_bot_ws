#include <PIDController.hpp>

void PIDController::setOutputLimits(int16_t minOutput, int16_t maxOutput) {
  m_minOutput = minOutput;
  m_maxOutput = maxOutput;
}
void PIDController::setPD(float Kp, float Kd) {
  m_Kp = Kp;
  m_Kd = Kd;
}

float PIDController::updatePD(float error, float measuredRate) {
  m_outputKp = m_Kp * error;
  m_outputKd = m_Kd * measuredRate;

  m_controlOutput = m_outputKp - m_outputKd;
  m_controlOutput = m_controlOutputRaw;
  if (m_controlOutput >= m_maxOutput)
    m_controlOutput = m_maxOutput;

  else if (m_controlOutput <= m_minOutput)
    m_controlOutput = m_minOutput;

  return m_controlOutput;
}

void PIDController::setP(float Kp) { m_Kp = Kp; }
void PIDController::setD(float Kd) { m_Kd = Kd; }