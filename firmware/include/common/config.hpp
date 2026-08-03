#pragma once

// #include <cstdint>
#include <stdint.h>

namespace config {

namespace math {
inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float microseconds_per_second = 1'000'000.0f;
} // namespace math

namespace encoder {
inline constexpr uint32_t timeout_us = 50'000;
inline constexpr float counts_per_revolution = 20.0f;
} // namespace encoder

namespace imu {
inline constexpr float complementary_filter_alpha = 0.98f;
inline constexpr uint32_t sample_period_us = 5'000;
} // namespace imu

namespace control {
inline constexpr float max_tilt_radians = 0.5f;
inline constexpr float target_angle_radians = 0.0f;
} // namespace control

namespace robot {
inline constexpr float wheel_radius_m = 0.0325f;
inline constexpr float axle_width_m = 0.15f;
} // namespace robot

} // namespace config
