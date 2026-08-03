#pragma once

enum class IMUState {
    INIT,
    CALIBRATING_GYRO,
    CALIBRATING_ZERO,
    RUNNING,
};
