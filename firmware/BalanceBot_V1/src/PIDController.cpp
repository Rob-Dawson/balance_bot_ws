#include <PIDController.hpp>

void PIDController::setOutputLimits(int8_t minOutput, int8_t maxOutput)
{
    m_minOutput = minOutput;
    m_maxOutput = maxOutput;
}
void PIDController::setPID(int Kp, int Ki, int Kd)
{
    m_Kp = Kp;
    m_Kd = Ki;
    m_Ki = Kd;
}

OutputLimits PIDController::getMinMaxOutput() const
{
    return {m_minOutput, m_maxOutput};
}

float PIDController::updatePD(float error, float measuredRate)
{
    m_outputKp = m_Kp * error;
    m_outputKd = m_Kd * measuredRate;

    m_controlOutput = m_outputKp - m_outputKd;

    if (m_controlOutput >= m_maxOutput)
        m_controlOutput = m_maxOutput;

    else if (m_controlOutput <= m_minOutput)
        m_controlOutput = m_minOutput;

    return m_controlOutput;
}