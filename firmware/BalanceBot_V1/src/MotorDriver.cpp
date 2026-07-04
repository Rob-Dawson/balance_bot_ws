#include "MotorDriver.hpp"
#include <Arduino.h>

void MotorDriver::init() {
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(m_motor1_input1, OUTPUT);
    pinMode(m_motor1_input2, OUTPUT);
    pinMode(m_motor2_input1, OUTPUT);
    pinMode(m_motor2_input2, OUTPUT);
}

int16_t MotorDriver::clamp(int16_t pwm) {
    if (pwm > m_MAX_PWM) {
        return m_MAX_PWM;
    } else if (pwm < -m_MAX_PWM) {
        return -m_MAX_PWM;
    }
    return pwm;
}

void MotorDriver::setDeadband(int deadband) { m_deadbandRight = deadband; }

void MotorDriver::setLeftPWM(int pwm) { m_requestedPWMLeft = pwm; }

void MotorDriver::setRightPWM(int pwm) { m_requestedPWMRight = pwm; }

void MotorDriver::setRightEffort(float controlInput) {

    m_requestedPWMRight = clamp((int16_t)round(controlInput));
}

void MotorDriver::setLeftEffort(float controlInput) {
    m_requestedPWMLeft = clamp((int16_t)round(controlInput));
}

void MotorDriver::moveRight() {
    int mapPWM = abs(m_requestedPWMRight);

    if (mapPWM == 0) {
        stopRight();
        return;
    }

    if (m_requestedPWMRight > 0) {
        digitalWrite(m_motor1_input1, HIGH);
        digitalWrite(m_motor1_input2, LOW);
        m_appliedPWMRight = map(mapPWM, 1, 255, m_deadbandRight, 255);
    } else {
        digitalWrite(m_motor1_input1, LOW);
        digitalWrite(m_motor1_input2, HIGH);
        m_appliedPWMRight = -map(mapPWM, 1, 255, m_deadbandRight, 255);
    }
    analogWrite(enA, abs(m_appliedPWMRight));
}

void MotorDriver::moveLeft() {
    int pwmMag = abs(m_requestedPWMLeft);

    if (pwmMag == 0) {
        stopLeft();
        return;
    }
    if (m_requestedPWMLeft > 0) {
        digitalWrite(m_motor2_input1, HIGH);
        digitalWrite(m_motor2_input2, LOW);
        m_appliedPWMLeft = map(pwmMag, 1, 255, m_deadbandLeft, 255);
    }

    else if (m_requestedPWMLeft < 0) {
        digitalWrite(m_motor2_input1, LOW);
        digitalWrite(m_motor2_input2, HIGH);
        m_appliedPWMLeft = -map(pwmMag, 1, 255, m_deadbandLeft, 255);
    }
    analogWrite(enB, abs(m_appliedPWMLeft));
}

void MotorDriver::stopLeft() {
    m_appliedPWMLeft = 0;
    digitalWrite(m_motor1_input1, LOW);
    digitalWrite(m_motor1_input2, LOW);
    analogWrite(enA, m_appliedPWMLeft);
}
void MotorDriver::stopRight() {
    m_appliedPWMRight = 0;
    digitalWrite(m_motor2_input1, LOW);
    digitalWrite(m_motor2_input2, LOW);
    analogWrite(enB, m_appliedPWMRight);
}