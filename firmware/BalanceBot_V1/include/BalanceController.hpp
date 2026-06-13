#include "PIDController.hpp"

class BalanceController
{
public:
    BalanceController();
    float update(float pitch, float pitchRate);
    float getError()const {return m_error;};
    float getTargetPitch()const {return m_targetPitch;};
    void init();

private:
    float m_error{};
    float m_targetPitch{};
    int8_t m_maxEffort{5};

    PIDController m_pitchController{};
};
