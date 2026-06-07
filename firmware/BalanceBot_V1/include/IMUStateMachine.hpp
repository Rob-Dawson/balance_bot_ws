#include "I2Cdev.h"
#include "MPU6050.h"
#include <Arduino.h>

enum class IMUState
{
    CALIBRATING_GYRO, 
    CALIBRATING_ZERO,
    RUNNING,
};

class IMUStateMachine
{
public:
    
    void update();
    void imuInit();
    IMUState getState();
    
private:
    IMUState state;
    void computeGyroBias();
    void computePitch();
    void computeZeroOffset();
    void imuConversions();


    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    float pitchEstimateSum = 0;
    float pitchOffset = 0;

    float pitchEstimate;
    float pitchRateEstimate;
    
    MPU6050 imu;
    int16_t ax,ay,az;
    int16_t gx,gy,gz;


    float accel_x_ms2 = 0.0;
    float accel_y_ms2 = 0.0;
    float accel_z_ms2 = 0.0;

    float gyro_x_rad = 0.0;
    float gyro_y_rad = 0.0;
    float gyro_z_rad = 0.0;


    float gxSum = 0.0;
    float gySum = 0.0;
    float gzSum = 0.0;


    uint16_t sampleCount = 0;

    unsigned long startTime = 0;
    unsigned long previousDTTime = 0;
    unsigned long startZeroTime = 0;

};

