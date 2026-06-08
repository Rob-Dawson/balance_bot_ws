#include "MotorDriver.hpp"
#include <Arduino.h>




void MotorDriver::init()
{
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(motor1_input1, OUTPUT);
    pinMode(motor1_input2, OUTPUT);
    pinMode(motor2_input1, OUTPUT);
    pinMode(motor2_input2, OUTPUT);
}

void MotorDriver::setLeftPWM(int pwm)
{
    //Forwards
    if (pwm > 0)
    {
        digitalWrite(motor1_input1, HIGH);
        digitalWrite(motor1_input2, LOW);
    }
    else if (pwm < 0)
    {
        digitalWrite(motor1_input1, LOW);
        digitalWrite(motor1_input2, HIGH);
    }
    analogWrite(enA, abs(pwm));
}

void MotorDriver::setRightPWM(int pwm)
{
    //Forwards
    if (pwm > 0)
    {
        digitalWrite(motor2_input1, HIGH);
        digitalWrite(motor2_input2, LOW);
    }
    
    //Backwards
    else if (pwm < 0)
    {
        digitalWrite(motor2_input1, LOW);
        digitalWrite(motor2_input2, HIGH);
    }
    analogWrite(enB, abs(pwm));
}