#include "IMUStateMachine.hpp"

void IMUStateMachine::imuInit()
{
    Wire.begin();
    imu.initialize();
    startTime = millis();
    state = IMUState::CALIBRATING_GYRO;
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

void IMUStateMachine::computePitch()
{
    if (previousDTTime == 0)
    {
        previousDTTime = micros();
    }
    unsigned long currentTime = micros();
    
    float dt = (currentTime - previousDTTime) / 1000000.0;
    previousDTTime = currentTime;
    float pitch = atan2(-accel_x_ms2,sqrt(accel_y_ms2*accel_y_ms2 + accel_z_ms2*accel_z_ms2));
    pitchRateEstimate = gyro_y_rad;
    pitchEstimate = 0.98 * (pitchEstimate + pitchRateEstimate * dt) + (1.0 - 0.98) * pitch;
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
        case IMUState::CALIBRATING_ZERO:
            computePitch();
            break;
    }
}