#include "encoder/ArduinoEncoder.hpp"
#include "common/config.hpp"
#include <Arduino.h>

namespace {
volatile Direction leftDirection{Direction::STOPPED};
volatile Direction rightDirection{Direction::STOPPED};

volatile uint32_t lastLhTime{0};
volatile uint32_t lastRhTime{0};
volatile uint32_t elapsedTimeLeft{0};
volatile uint32_t elapsedTimeRight{0};

constexpr uint8_t lhEncoderA{3};
constexpr uint8_t lhEncoderB{13};

constexpr uint8_t rhEncoderA{2};
constexpr uint8_t rhEncoderB{8};

void leftEncoderEvent() {
    const uint32_t currentTime = micros();
    if (lastLhTime == 0) {
        lastLhTime = currentTime;
        return;
    }
    elapsedTimeLeft = currentTime - lastLhTime;
    lastLhTime = currentTime;

    // A quadrature encoder cycles through the following states:
    //
    //   00 → 01 → 11 → 10 → 00
    //
    // This implementation does not decode every transition.
    // Instead, an interrupt is generated whenever channel A changes.
    // At that instant, channel B is sampled to determine the direction:
    //
    //   A == B : Forward
    //   A != B : Reverse
    //
    // This provides one direction update for each transition on channel A.
    const uint8_t encoderA = digitalRead(lhEncoderA);
    const uint8_t encoderB = digitalRead(lhEncoderB);

    if (encoderA != encoderB) {
        leftDirection = Direction::REVERSE;
    } else {
        leftDirection = Direction::FORWARD;
    }
}
void rightEncoderEvent() {
    const uint32_t currentTime = micros();
    if (lastRhTime == 0) {
        lastRhTime = currentTime;
        return;
    }

    elapsedTimeRight = currentTime - lastRhTime;
    lastRhTime = currentTime;

    const uint8_t encoderA = digitalRead(rhEncoderA);
    const uint8_t encoderB = digitalRead(rhEncoderB);

    if (encoderA != encoderB) {
        rightDirection = Direction::REVERSE;
    } else {
        rightDirection = Direction::FORWARD;
    }
}

} // namespace

float ArduinoEncoder::getSpeedLeft() {
    uint32_t elapsed;
    uint32_t lastTime;
    Direction direction;

    noInterrupts();
    elapsed = elapsedTimeLeft;
    lastTime = lastLhTime;
    direction = leftDirection;

    interrupts();

    if (elapsed == 0 || micros() - lastTime > config::encoder::timeout_us) {
        return 0.0f;
    }

    return EncoderMath::calculateWheelSpeed(elapsed, direction);
}

float ArduinoEncoder::getSpeedRight() {
    uint32_t elapsed;
    uint32_t lastTime;
    Direction direction;
    noInterrupts();
    elapsed = elapsedTimeRight;
    lastTime = lastRhTime;
    direction = rightDirection;
    interrupts();

    if (elapsed == 0 || micros() - lastTime > config::encoder::timeout_us) {
        return 0.0f;
    }

    return EncoderMath::calculateWheelSpeed(elapsed, direction);
}

bool ArduinoEncoder::init() {
    pinMode(lhEncoderA, INPUT);
    pinMode(lhEncoderB, INPUT);
    pinMode(rhEncoderA, INPUT);
    pinMode(rhEncoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(lhEncoderA), leftEncoderEvent,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(rhEncoderA), rightEncoderEvent,
                    CHANGE);
    return true;
}
