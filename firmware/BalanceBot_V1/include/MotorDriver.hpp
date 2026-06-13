#include <Arduino.h>

class MotorDriver
{
public:
    void init(); 
    void stop();
    void setLeftPWM(float controlInput);
    void setRightPWM(float controlInput);

private:
    int16_t clamp(int16_t);

    const uint8_t enA = 5;
    const uint8_t m_motor1_input1 = 6;
    const uint8_t m_motor1_input2 = 7;

    const uint8_t m_motor2_input1 = 4;
    const uint8_t m_motor2_input2 = 10;
    const uint8_t enB = 11;

    static constexpr int16_t m_MAX_PWM = 255;
    static constexpr int8_t m_deadband = 40;
};