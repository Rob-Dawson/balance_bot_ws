#pragma once

#include "IIMU.hpp"
#include "MPU6050.h"
#include <Arduino.h>
class IMUStateMachine : public IIMU {
public:
    void update() override;
    bool init() override;
    IMUState getState() const override { return m_state; };
    float getPitch() const override { return m_pitchEstimate - m_pitchOffset; };
    float getPitchRate() const override { return m_pitchRateEstimate; };
    float getRawPitch() const override { return m_pitchEstimate; };

private:
    IMUState m_state{IMUState::INIT};
    void computeGyroBias();
    void computePitch();
    void computeZeroOffset();
    void imuConversions();

    float m_gyroBiasX{};
    float m_gyroBiasY{};
    float m_gyroBiasZ{};

    float m_pitchOffset{};
    float m_pitchEstimateSum{};

    bool m_pitchInit{false};
    float m_pitchEstimate{};
    float m_pitchRateEstimate{};

    MPU6050 m_imu;
    int16_t m_ax{}, m_ay{}, m_az{};
    int16_t m_gx{}, m_gy{}, m_gz{};

    float m_accelXMs2{};
    float m_accelYMs2{};
    float m_accelZMs2{};

    float m_gyroXRad{};
    float m_gyroYRad{};
    float m_gyroZRad{};

    float m_gxSum{};
    float m_gySum{};
    float m_gzSum{};
    const float m_alpha{0.98F};
    uint16_t m_sampleCount{};

    unsigned long m_startTime{};
    unsigned long m_previousDTTime{};
    unsigned long m_startZeroTime{};
};