#include <Arduino.h>

namespace {
    volatile int8_t leftDirection = 0;
    volatile int8_t rightDirection = 0;

    volatile uint32_t lastLhTime = 0;
    volatile uint32_t lastRhTime = 0;
    volatile uint32_t elapsedTimeLeft = 0;
    volatile uint32_t elapsedTimeRight = 0;

    constexpr uint8_t lhEncoderA = 3;
    constexpr uint8_t lhEncoderB = 13;

    constexpr uint8_t rhEncoderA = 2;
    constexpr uint8_t rhEncoderB = 8;
}
class Encoder
{
public:
    void init();
    float getSpeedRight();
    float getSpeedLeft();
    int8_t getLeftDirection();
    int8_t getRightDirection();


private:

    static constexpr float m_countsPerRev = 1000.0f;
    static constexpr uint32_t m_timeout = 50000; //50 ms

    static constexpr float m_micro2Sec = 1000000.0f;

};