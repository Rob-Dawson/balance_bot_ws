#include "Encoder.hpp"


volatile unsigned long lastLhTime = 0;
volatile unsigned long lastRhTime = 0;
volatile unsigned long elapsedTimeLeft;
volatile unsigned long elapsedTimeRight;
static void leftEncoderEvent()
{
    unsigned long currentTime = micros();
    if (lastLhTime == 0)
    {
        lastLhTime = currentTime;
        return;
    }
    elapsedTimeLeft = currentTime - lastLhTime;

    lastLhTime = currentTime;
}

static void rightEncoderEvent()
{
    unsigned long currentTime = micros();
    if (lastRhTime == 0)
    {
        lastRhTime = currentTime;
        return;
    }
    elapsedTimeRight = currentTime - lastRhTime;
    lastRhTime = currentTime;
}

float Encoder::getSpeedLeft()
{
    if (micros() - lastLhTime > timeout) return 0.0f;
    if (elapsedTimeLeft == 0) return 0.0f; 

    float countsPerSecond = 1000000/elapsedTimeLeft;
    wheelRadPerSec = countsPerSecond * (2*PI / countsPerRev);
    return wheelRadPerSec;
}

float Encoder::getSpeedRight()
{
    if (micros() - lastRhTime > timeout) return 0.0f;
    if (elapsedTimeRight == 0) return 0.0f; 
    float countsPerSecond = 1000000/elapsedTimeRight;
    wheelRadPerSec = countsPerSecond * (2*PI / countsPerRev);
    return wheelRadPerSec;
}

void Encoder::init()
{
    pinMode(lh_encoder_A, INPUT);
    pinMode(lh_encoder_B, INPUT);
    pinMode(rh_encoder_A, INPUT);
    pinMode(rh_encoder_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(lh_encoder_A), leftEncoderEvent, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rh_encoder_A), rightEncoderEvent, CHANGE);
}