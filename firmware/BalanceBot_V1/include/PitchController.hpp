#include "PIDController.hpp"
#include <Arduino.h>

class PitchController {
public:
    PitchController();
    float update(float pitchSetpoint, float pitch, float pitchRate);

    float getError() const { return m_error; };

    float getOutputKp() const { return m_pitchController.getKp(); };
    float getOutputKd() const { return m_pitchController.getKd(); };

    void setKp(float Kp);
    void setKd(float Kd);

    void init();

private:
    float m_error{};
    int16_t m_maxEffort{255};

    PIDController m_pitchController{};
};
