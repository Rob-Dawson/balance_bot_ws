#include "IMUStateMachine.hpp"
#include <stdint.h>

#define DEBUG_PRINTLN(x) Serial.println(x)
namespace {
constexpr float Gravity{9.80665F};
constexpr float AccelLsbPerg{16384.0F};
constexpr float GyroLsbPerDegPerSec{131.0F};
constexpr float DegToRad = PI / 180.0F;
constexpr int gyroCalibrationTime{2000};
constexpr float MicroToSeconds = 1.0e-6F;
} // namespace

void IMUStateMachine::imuConversions() {
    m_accelXMs2 = m_ax / AccelLsbPerg * Gravity;
    m_accelYMs2 = m_ay / AccelLsbPerg * Gravity;
    m_accelZMs2 = m_az / AccelLsbPerg * Gravity;

    m_gyroXRad = ((m_gx / GyroLsbPerDegPerSec) * DegToRad) - m_gyroBiasX;
    m_gyroYRad = ((m_gy / GyroLsbPerDegPerSec) * DegToRad) - m_gyroBiasY;
    m_gyroZRad = ((m_gz / GyroLsbPerDegPerSec) * DegToRad) - m_gyroBiasZ;
}

void IMUStateMachine::imuInit() {
    Wire.begin();
    m_imu.initialize();
    m_startTime = millis();
    m_state = IMUState::CALIBRATING_GYRO;
    Serial.println(m_imu.testConnection() ? "MPU6050 connection successful"
                                          : "MPU6050 connection failed");
}

void IMUStateMachine::computeGyroBias() {
    m_gxSum += m_gyroXRad;
    m_gySum += m_gyroYRad;
    m_gzSum += m_gyroZRad;
    m_sampleCount++;

    if (millis() - m_startTime > gyroCalibrationTime) {
        m_gyroBiasX = m_gxSum / m_sampleCount;
        m_gyroBiasY = m_gySum / m_sampleCount;
        m_gyroBiasZ = m_gzSum / m_sampleCount;
        m_sampleCount = 0;
        m_state = IMUState::CALIBRATING_ZERO;
    }
}

void IMUStateMachine::computePitch() {
    if (m_previousDTTime == 0) {
        m_previousDTTime = micros();
    }

    const uint32_t currentTime = micros();
    const uint32_t dtUs = currentTime - m_previousDTTime;
    m_previousDTTime = currentTime;
    const float dt = static_cast<float>(dtUs) * MicroToSeconds;
    m_previousDTTime = currentTime;

    float pitchAngle = atan2f(m_accelXMs2, sqrtf(m_accelYMs2 * m_accelYMs2 +
                                                 m_accelZMs2 * m_accelZMs2));
    if (!m_pitchInit) {
        m_pitchEstimate = pitchAngle;
        m_pitchInit = true;
    }

    m_pitchRateEstimate = m_gyroYRad;
    m_pitchEstimate = m_alpha * (m_pitchEstimate + m_pitchRateEstimate * dt) +
                      (1.0f - m_alpha) * pitchAngle;
}

void IMUStateMachine::computeZeroOffset() {
    if (m_startZeroTime == 0) {
        m_startZeroTime = millis();
    }
    m_pitchEstimateSum += m_pitchEstimate;
    m_sampleCount++;
    if (millis() - m_startZeroTime > gyroCalibrationTime) {
        m_pitchOffset = m_pitchEstimateSum / m_sampleCount;
        m_sampleCount = 0;
        m_state = IMUState::RUNNING;
    }
}

void IMUStateMachine::update() {
    m_imu.getMotion6(&m_ax, &m_ay, &m_az, &m_gx, &m_gy, &m_gz);
    imuConversions();

    switch (m_state) {
    case IMUState::INIT:
        break;
    case IMUState::CALIBRATING_GYRO:
        computeGyroBias();
        break;
    case IMUState::CALIBRATING_ZERO:
        computePitch();
        computeZeroOffset();
        break;
    case IMUState::RUNNING:
        computePitch();
        break;
    }
}
