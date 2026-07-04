#include <Arduino.h>

class MotorDriver {
public:
    void init();
    void stopLeft();
    void stopRight();

    void setLeftPWM(int pwm);
    void setRightPWM(int pwm);
    void setLeftEffort(float controlInput);
    void setRightEffort(float controlInput);
    void moveLeft();
    void moveRight();
    void setDeadband(int deadband);

    int16_t getRequestedPWMLeft() { return m_requestedPWMRight; };
    int16_t getRequestedPWMRight() { return m_requestedPWMLeft; };

    int16_t getAppliedPWMRight() { return m_appliedPWMRight; };
    int16_t getAppliedPWMLeft() { return m_appliedPWMLeft; };

private:
    int16_t clamp(int16_t);

    int16_t m_requestedPWMLeft{};
    int16_t m_requestedPWMRight{};

    int16_t m_appliedPWMLeft{};
    int16_t m_appliedPWMRight{};

    const uint8_t enA{5};
    const uint8_t m_motor1_input1{11};
    const uint8_t m_motor1_input2{7};

    const uint8_t m_motor2_input1{4};
    const uint8_t m_motor2_input2{10};
    const uint8_t enB{6};

    static constexpr int16_t m_MAX_PWM{255};
    int16_t m_deadbandRight{45};
    int16_t m_deadbandLeft{52};
};