#include "Arduino.h"

struct OutputLimits
{
    int8_t minOutput;
    int8_t maxOutput;
};

class PIDController
{
public:
    // float updatePI(float error, unsigned long dt);
    float updatePD(float error, float measuredRate);

    void setPID(int Kp, int Ki, int Kd);
    void setOutputLimits(int8_t maxRate, int8_t minRate);
    OutputLimits getMinMaxOutput() const;

private: 
    float m_Kp{};
    float m_Kd{};
    float m_Ki{};
    float m_measuredRate{};  // <- Pitch Rate
    float m_error{}; // <- error = self.setpoint - self.imu_pitch

    float m_outputKp{};
    float m_outputKi{};
    float m_outputKd{};

    float m_controlOutput{};
    
    int8_t m_maxOutput{};
    int8_t m_minOutput{};
    
};