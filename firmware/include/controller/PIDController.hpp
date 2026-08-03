#pragma once

struct OutputLimits {
    float minOutput;
    float maxOutput;
};

class PIDController {
public:
    float updatePD(float error, float measuredRate);
    float updatePI(float error, float dt);
    float updatePID(float error, float measuredRate, float dt);

    void setPD(float Kp, float Kd);
    void setPI(float Kp, float Ki);
    void setPID(float Kp, float Ki, float Kd);
    void setP(float Kp);
    void setI(float Ki);
    void setD(float Kd);

    void setOutputLimits(float minOutput, float maxOutput);
    OutputLimits getMinMaxOutput() const { return {m_minOutput, m_maxOutput}; };

    float getKp() const { return m_outputKp; };
    float getKi() const { return m_outputKi; };
    float getKd() const { return m_outputKd; };

    float getRawOutput() const { return m_controlOutputRaw; };

private:
    float m_Kp{};
    float m_Ki{};
    float m_Kd{};
    float m_measuredRate{}; // Pitch Rate
    float m_error{};        // error = self.setpoint - self.imu_pitch
    float m_outputKp{};
    float m_outputKi{};
    float m_outputKd{};
    float m_integral{};

    float m_controlOutput{};
    float m_controlOutputRaw{};

    float m_maxOutput{};
    float m_minOutput{};
    const float MaxIntegrationDt{0.01F};
};