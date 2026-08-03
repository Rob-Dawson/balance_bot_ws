#pragma once
#include "../common/config.hpp"
// #include <cstdint>

enum class Direction : int8_t {
    REVERSE = -1,
    STOPPED = 0,
    FORWARD = 1,

};

namespace EncoderMath {
inline float calculateWheelSpeed(uint32_t elapsed, Direction direction) {
    float encoderRateHz =
        config::math::microseconds_per_second / static_cast<float>(elapsed);

    float wheelRadPerSec =
        encoderRateHz *
        (2 * config::math::pi / config::encoder::counts_per_revolution);
    wheelRadPerSec = wheelRadPerSec * static_cast<int8_t>(direction);
    return wheelRadPerSec;
}

} // namespace EncoderMath
