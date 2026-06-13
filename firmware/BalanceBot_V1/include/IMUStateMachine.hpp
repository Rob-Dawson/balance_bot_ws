#include "I2Cdev.h"
#include "MPU6050.h"
#include <Arduino.h>

enum class IMUState
{
    INIT,
    CALIBRATING_GYRO, 
    CALIBRATING_ZERO,
    RUNNING,
};

class IMUStateMachine
{
public:
    
    void update();
    void imuInit();
    IMUState getState() const;
    float getPitch() const;
    float getPitchRate() const;
    float getRawPitch() const;



private:

    IMUState state = IMUState::INIT;
    void computeGyroBias();
    void computePitch();
    void computeZeroOffset();
    void imuConversions();

    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    float pitchOffset{};
    float pitchEstimateSum{};

    float pitchEstimate;
    float pitchRateEstimate;

    
    
    MPU6050 imu;
    int16_t ax,ay,az{};
    int16_t gx,gy,gz{};

    float accel_x_ms2{};
    float accel_y_ms2{};
    float accel_z_ms2{};

    float gyro_x_rad{};
    float gyro_y_rad{};
    float gyro_z_rad{};

    float gxSum{};
    float gySum{};
    float gzSum{};
    uint16_t sampleCount;

    unsigned long startTime{};
    unsigned long previousDTTime{};
    unsigned long startZeroTime{};

};