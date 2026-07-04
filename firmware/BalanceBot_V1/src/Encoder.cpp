#include "Encoder.hpp"

// 1 = Forward
// -1 = Backward

static void leftEncoderEvent() {
    uint32_t currentTime = micros();
    if (lastLhTime == 0) {
        lastLhTime = currentTime;
        return;
    }
    elapsedTimeLeft = currentTime - lastLhTime;
    lastLhTime = currentTime;

    // This function fires only if encoderA changes
    // 00
    // 01
    // 11
    // 10
    // 00

    // This function will fire at
    // 00 11 00
    int8_t encoderA = digitalRead(lhEncoderA);
    int8_t encoderB = digitalRead(lhEncoderB);

    if (encoderA != encoderB) {
        leftDirection = -1;
    } else {
        leftDirection = 1;
    }
}
static void rightEncoderEvent() {
    uint32_t currentTime = micros();
    if (lastRhTime == 0) {
        lastRhTime = currentTime;
        return;
    }

    elapsedTimeRight = currentTime - lastRhTime;
    lastRhTime = currentTime;

    int8_t encoderA = digitalRead(rhEncoderA);
    int8_t encoderB = digitalRead(rhEncoderB);

    if (encoderA != encoderB) {
        rightDirection = -1;
    } else {
        rightDirection = 1;
    }
}

float Encoder::getSpeedLeft() {
    uint32_t elapsed;
    uint32_t lastTime;

    noInterrupts();
    elapsed = elapsedTimeLeft;
    lastTime = lastLhTime;
    interrupts();

    if (micros() - lastTime > m_timeout)
        return 0.0f;
    if (elapsed == 0)
        return 0.0f;

    float countsPerSecond = m_micro2Sec / elapsed;
    float wheelRadPerSec = countsPerSecond * (2 * PI / m_countsPerRev);
    wheelRadPerSec = wheelRadPerSec * Encoder::getLeftDirection();
    return wheelRadPerSec;
}

float Encoder::getSpeedRight() {
    uint32_t elapsed;
    uint32_t lastTime;
    int8_t direction;

    noInterrupts();
    elapsed = elapsedTimeRight;
    lastTime = lastRhTime;
    interrupts();

    if (micros() - lastTime > m_timeout)
        return 0.0f;
    if (elapsed == 0)
        return 0.0f;
    float countsPerSecond = m_micro2Sec / elapsed;
    float wheelRadPerSec = countsPerSecond * (2 * PI / m_countsPerRev);

    wheelRadPerSec = wheelRadPerSec * Encoder::getRightDirection();
    return wheelRadPerSec;
}

int8_t Encoder::getLeftDirection() { return leftDirection; }

int8_t Encoder::getRightDirection() { return rightDirection; }

void Encoder::init() {
    pinMode(lhEncoderA, INPUT);
    pinMode(lhEncoderB, INPUT);
    pinMode(rhEncoderA, INPUT);
    pinMode(rhEncoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(lhEncoderA), leftEncoderEvent,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(rhEncoderA), rightEncoderEvent,
                    CHANGE);
}