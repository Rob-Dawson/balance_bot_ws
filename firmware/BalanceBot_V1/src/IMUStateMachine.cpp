#include "IMUStateMachine.hpp"
#define DEBUG_PRINTLN(x) Serial.println(x)

void IMUStateMachine::imuConversions() {
    m_accelXMs2 = m_ax / 16384.0 * 9.80665;
    m_accelYMs2 = m_ay / 16384.0 * 9.80665;
    m_accelZMs2 = m_az / 16384.0 * 9.80665;

    m_gyroXRad = ((m_gx / 131.0f) * PI / 180.0f) - m_gyroBiasX;
    m_gyroYRad = ((m_gy / 131.0f) * PI / 180.0f) - m_gyroBiasY;
    m_gyroZRad = ((m_gz / 131.0f) * PI / 180.0f) - m_gyroBiasZ;
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

    if (millis() - m_startTime > 2000.0) {
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

    unsigned long currentTime = micros();
    float dt = (currentTime - m_previousDTTime) / 1000000.0;
    m_previousDTTime = currentTime;

    float pitchAngle = atan2(m_accelXMs2, sqrt(m_accelYMs2 * m_accelYMs2 +
                                               m_accelZMs2 * m_accelZMs2));
    if (!m_pitchInit) {
        m_pitchEstimate = pitchAngle;
        m_pitchInit = true;
    }

    m_pitchRateEstimate = m_gyroYRad;
    m_pitchEstimate = 0.98f * (m_pitchEstimate + m_pitchRateEstimate * dt) +
                      (1.0 - 0.98f) * pitchAngle;
}

void IMUStateMachine::computeZeroOffset() {
    if (m_startZeroTime == 0) {
        m_startZeroTime = millis();
    }
    m_pitchEstimateSum += m_pitchEstimate;
    m_sampleCount++;
    if (millis() - m_startZeroTime > 2000.0) {
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
