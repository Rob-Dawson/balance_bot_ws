#include "controller/PitchController.hpp"

PitchController::PitchController() {
    m_pitchController.setOutputLimits(-255, 255);
    m_pitchController.setPD(1000.0, 150.0);
}

void PitchController::setKp(float Kp) { m_pitchController.setP(Kp); }

void PitchController::setKd(float Kd) { m_pitchController.setD(Kd); }

float PitchController::update(float pitchSetpoint, float pitch,
                              float pitchRate) {
    m_targetPitch = pitchSetpoint;
    m_error = m_targetPitch - pitch;
    float controlOutput = m_pitchController.updatePD(m_error, pitchRate);
    return controlOutput;
}
