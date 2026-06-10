#include "MotorDriver.hpp"
#include <Arduino.h>

void MotorDriver::init()
{
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(m_motor1_input1, OUTPUT);
    pinMode(m_motor1_input2, OUTPUT);
    pinMode(m_motor2_input1, OUTPUT);
    pinMode(m_motor2_input2, OUTPUT);
}

int16_t MotorDriver::clamp(int16_t pwm)
{
    if (pwm > m_MAX_PWM)
    {
        return m_MAX_PWM;
    }
    else if(pwm < -m_MAX_PWM)
    {
        return -m_MAX_PWM;
    }
    return pwm;
}

void MotorDriver::setLeftPWM(int16_t pwm)
{
    pwm = clamp(pwm);
    //Forwards

    if (pwm > 0)
    {
        digitalWrite(m_motor1_input1, LOW);
        digitalWrite(m_motor1_input2, HIGH);

    }
    else if (pwm < 0)
    {
        digitalWrite(m_motor1_input1, HIGH);
        digitalWrite(m_motor1_input2, LOW);
    }
    analogWrite(enA, abs(pwm));
}

void MotorDriver::setRightPWM(int16_t pwm)
{
    pwm = clamp(pwm);
    //Forwards
    if (pwm > 0)
    {
        digitalWrite(m_motor2_input1, HIGH);
        digitalWrite(m_motor2_input2, LOW);
    }
    
    //Backwards
    else if (pwm < 0)
    {
        digitalWrite(m_motor2_input1, LOW);
        digitalWrite(m_motor2_input2, HIGH);
    }
    
    analogWrite(enB, abs(pwm));
}

void MotorDriver::stop()
{
    digitalWrite(m_motor1_input1, LOW);
    digitalWrite(m_motor1_input2, LOW);

    digitalWrite(m_motor2_input1, LOW);
    digitalWrite(m_motor2_input2, LOW);

    analogWrite(enA, 0);
    analogWrite(enB, 0);
}