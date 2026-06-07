#include "IMUStateMachine.hpp"

void IMUStateMachine::imuInit()
{
    Wire.begin();
    imu.initialize();
    startTime = millis();
    state = IMUState::CALIBRATING_GYRO;
}

void IMUStateMachine::imuConversions()
{
    accel_x_ms2 = ax / 16384.0 * 9.80665;
    accel_y_ms2 = ay / 16384.0 * 9.80665;
    accel_z_ms2 = az / 16384.0 * 9.80665;
    
    gyro_x_rad = ((gx / 131.0f) * PI / 180.0f);
    gyro_y_rad = ((gy / 131.0f) * PI / 180.0f);
    gyro_z_rad = ((gz / 131.0f) * PI / 180.0f);
}

void IMUStateMachine::computeGyroBias()
{
    gxSum += gyro_x_rad;
    gySum += gyro_y_rad;
    gzSum += gyro_z_rad;
    sampleCount++;

    if (millis() - startTime > 2000.0)
    {
        gyroBiasX = gxSum / sampleCount;
        gyroBiasY = gySum / sampleCount;
        gyroBiasZ = gzSum / sampleCount;
        sampleCount = 0;
        state = IMUState::CALIBRATING_ZERO;
    }
}

void IMUStateMachine::update()
{
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    imuConversions();

    switch(state)
    {
        case IMUState::CALIBRATING_GYRO:
            computeGyroBias();
            break;
    }
}