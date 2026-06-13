#include "BalanceController.hpp"

BalanceController::BalanceController()
{
    m_pitchController.setOutputLimits(-m_maxEffort, m_maxEffort);
    m_pitchController.setPID(5.5,0.9,1.0);
}

    float BalanceController::update(float pitch, float pitchRate)
{
    m_error = m_targetPitch - pitch;
    Serial.print("ERROR: ");
    Serial.print(m_error);

    float controlOutput = m_pitchController.updatePD(m_error, pitchRate);
    Serial.print("\t\tControl OUTPUT:  ");
    Serial.println(controlOutput);

    return controlOutput;
}
