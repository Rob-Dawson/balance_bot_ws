#include "Arduino.h"

struct OutputLimits {
  int16_t minOutput;
  int16_t maxOutput;
};

class PIDController {
public:
  // float updatePI(float error, unsigned long dt);
  float updatePD(float error, float measuredRate);

  void setPD(float Kp, float Kd);
  void setP(float Kp);
  void setD(float Kd);

  void setOutputLimits(int16_t maxRate, int16_t minRate);
  OutputLimits getMinMaxOutput() const { return {m_minOutput, m_maxOutput}; };

  float getKp() const { return m_outputKp; };
  float getKd() const { return m_outputKd; };
  float getRawOutput() const { return m_controlOutputRaw; };

private:
  float m_Kp{};
  float m_Kd{};
  float m_measuredRate{}; // Pitch Rate
  float m_error{};        // error = self.setpoint - self.imu_pitch

  float m_outputKp{};
  float m_outputKd{};

  float m_controlOutput{};
  float m_controlOutputRaw{};

  int16_t m_maxOutput{};
  int16_t m_minOutput{};
};