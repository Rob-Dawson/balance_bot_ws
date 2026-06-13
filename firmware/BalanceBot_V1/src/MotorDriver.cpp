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

void MotorDriver::setLeftPWM(float controlInput)
{
    float pwmFloat = controlInput * 51;

    int16_t pwmInt = round(pwmFloat);
    pwmInt = clamp(pwmInt);
    Serial.print("\t\t\tPWM: ");
    Serial.println(pwmInt);

    //Forwards

    if (pwmInt > 0)
    {
        digitalWrite(m_motor1_input1, LOW);
        digitalWrite(m_motor1_input2, HIGH);
        pwmInt = pwmInt + m_deadband;
    }
    else if (pwmInt < 0)
    {
        digitalWrite(m_motor1_input1, HIGH);
        digitalWrite(m_motor1_input2, LOW);
        pwmInt = pwmInt - m_deadband;

    }
    pwmInt = clamp(pwmInt);
    
    analogWrite(enA, abs(pwmInt));
}

void MotorDriver::setRightPWM(float controlInput)
{
    float pwmFloat = controlInput * 51;
    int16_t pwmInt = round(pwmFloat);
    pwmInt = clamp(pwmInt);
    Serial.print("\t\t\tPWM: ");
    Serial.println(pwmInt);
    
    //Forwards
    if (pwmInt > 0)
    {
        digitalWrite(m_motor2_input1, HIGH);
        digitalWrite(m_motor2_input2, LOW);
        pwmInt = pwmInt + m_deadband;

    }
    
    //Backwards
    else if (pwmInt < 0)
    {
        digitalWrite(m_motor2_input1, LOW);
        digitalWrite(m_motor2_input2, HIGH);
        pwmInt = pwmInt - m_deadband;
    }
    pwmInt = clamp(pwmInt);
    
    analogWrite(enB, abs(pwmInt));
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