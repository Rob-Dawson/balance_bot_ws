#include <Arduino.h>


class Encoder
{
public:
    void init();
    float getSpeedRight();
    float getSpeedLeft();

private:

    const float countsPerRev = 1000.0f;
    float wheelRadPerSec = 0.0;


    const int lh_encoder_A = 3;
    const int lh_encoder_B = 13;

    const int rh_encoder_A = 2;
    const int rh_encoder_B = 8;
    const unsigned int timeout = 50000; //50 ms

};