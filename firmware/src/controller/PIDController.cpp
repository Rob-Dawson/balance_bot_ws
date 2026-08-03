#include <controller/PIDController.hpp>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void PIDController::setOutputLimits(float minOutput, float maxOutput) {
    m_minOutput = minOutput;
    m_maxOutput = maxOutput;
}

void PIDController::setPD(float Kp, float Kd) {
    m_Kp = Kp;
    m_Kd = Kd;
    m_Ki = 0;
}

void PIDController::setPI(float Kp, float Ki) {
    m_Kp = Kp;
    m_Kd = 0;
    m_Ki = Ki;
}

void PIDController::setP(float Kp) { m_Kp = Kp; }
void PIDController::setI(float Ki) { m_Ki = Ki; }
void PIDController::setD(float Kd) { m_Kd = Kd; }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
float PIDController::updatePD(float error, float measuredRate) {
    m_outputKp = m_Kp * error;
    m_outputKi = 0;
    m_outputKd = m_Kd * measuredRate;
    float outputRaw = m_outputKp - m_outputKd;
    m_controlOutput = outputRaw;

    if (m_controlOutput >= m_maxOutput) {
        m_controlOutput = m_maxOutput;
    } else if (m_controlOutput <= m_minOutput) {
        m_controlOutput = m_minOutput;
    }

    return m_controlOutput;
}

float PIDController::updatePI(float error, float dt) {
    m_outputKp = m_Kp * error;
    m_outputKi = m_Ki * m_integral;
    m_outputKd = 0;
    float rawOutput = m_outputKp + m_outputKi;
    if (m_Ki != 0) {
        bool outputWithinLimits =
            rawOutput >= m_minOutput && rawOutput < m_maxOutput;

        bool reducingHighSaturation = rawOutput > m_maxOutput && error < 0;

        bool reducingLowSaturation = rawOutput < m_minOutput && error > 0;

        if (m_Ki != 0 && dt > 0.0F && dt < MaxIntegrationDt &&
            (outputWithinLimits || reducingHighSaturation ||
             reducingLowSaturation)) {
            m_integral += error * dt;
        }
    }
    m_outputKi = m_Ki * m_integral;
    m_controlOutput = m_outputKp + m_outputKi;
    m_controlOutputRaw = m_controlOutput;

    if (m_controlOutput >= m_maxOutput) {
        m_controlOutput = m_maxOutput;

    } else if (m_controlOutput <= m_minOutput) {
        m_controlOutput = m_minOutput;
    }

    return m_controlOutput;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
float PIDController::updatePID(float error, float measuredRate, float dt) {
    m_outputKp = m_Kp * error;
    m_outputKd = m_Kd * measuredRate;
    m_outputKi = m_Ki * m_integral;
    float rawOutput = m_outputKp + m_outputKi - m_outputKd;
    bool outputWithinLimits =
        rawOutput >= m_minOutput && rawOutput < m_maxOutput;
    bool reducingHighSaturation = rawOutput > m_maxOutput && error < 0;
    bool reducingLowSaturation = rawOutput < m_minOutput && error > 0;
    bool shouldIntegrate =
        outputWithinLimits || reducingHighSaturation || reducingLowSaturation;

    if (m_Ki != 0 && dt > 0.0F && dt < MaxIntegrationDt && shouldIntegrate) {
        m_integral += error * dt;
    }

    m_outputKi = m_Ki * m_integral;
    m_controlOutput = m_outputKp + m_outputKi - m_outputKd;
    m_controlOutputRaw = m_controlOutput;

    if (m_controlOutput >= m_maxOutput) {
        m_controlOutput = m_maxOutput;

    } else if (m_controlOutput <= m_minOutput) {
        m_controlOutput = m_minOutput;
    }

    return m_controlOutput;
}
