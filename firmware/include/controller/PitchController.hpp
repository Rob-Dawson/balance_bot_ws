#pragma once

#include "PIDController.hpp"
#include <Arduino.h>

class PitchController {
public:
    PitchController();
    float update(float pitch, float pitchRate, float dt);

    float getError() const { return m_error; };
    float getTargetPitch() const { return m_targetPitch; };

    float getOutputKp() const { return m_pitchController.getKp(); };
    float getOutputKd() const { return m_pitchController.getKd(); };

    void setKp(float Kp);
    void setKd(float Kd);

    void init();

private:
    float m_error{};
    float m_targetPitch{0.0};
    const int16_t m_maxEffort{255};

    PIDController m_pitchController{};
};
