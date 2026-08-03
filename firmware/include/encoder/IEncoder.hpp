#pragma once
#include "EncoderMath.hpp"
#include <stdint.h>

// #include <cstdint>

class IEncoder {
public:
    virtual ~IEncoder() = default;
    virtual bool init() = 0;
    virtual float getSpeedRight() = 0;
    virtual float getSpeedLeft() = 0;
    // virtual Direction getLeftDirection() = 0;
    // virtual Direction getRightDirection() = 0;
};
