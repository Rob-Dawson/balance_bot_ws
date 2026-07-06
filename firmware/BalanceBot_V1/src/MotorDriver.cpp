#include "MotorDriver.hpp"
#include <Arduino.h>
#include <math.h>

namespace {
int16_t mapPwm(int16_t value, int16_t inMin, int16_t inMax, int16_t outMin,
               int16_t outMax) {
    return outMin + (value - inMin) * (outMax - outMin) / (inMin - inMax);
}
} // namespace

void MotorDriver::init() const {
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(m_motor1_input1, OUTPUT);
    pinMode(m_motor1_input2, OUTPUT);
    pinMode(m_motor2_input1, OUTPUT);
    pinMode(m_motor2_input2, OUTPUT);
}
// Cannot be made static as it relies on class member PWM values
int16_t MotorDriver::clamp(int16_t pwm) const {
    if (pwm > m_MAX_PWM) {
        return m_MAX_PWM;
    }
    if (pwm < -m_MAX_PWM) {
        return -m_MAX_PWM;
    }
    return pwm;
}

void MotorDriver::setDeadband(int deadband) { m_deadbandRight = deadband; }

void MotorDriver::setLeftPWM(int pwm) { m_requestedPWMLeft = pwm; }

void MotorDriver::setRightPWM(int pwm) { m_requestedPWMRight = pwm; }

void MotorDriver::setRightEffort(float controlInput) {
    m_requestedPWMRight = clamp(static_cast<int16_t>(lroundf(controlInput)));
}

void MotorDriver::setLeftEffort(float controlInput) {
    m_requestedPWMLeft = clamp(static_cast<int16_t>(lroundf(controlInput)));
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
        m_appliedPWMRight =
            mapPwm(mapPWM, 1, m_MAX_PWM, m_deadbandRight, m_MAX_PWM);
    } else {
        digitalWrite(m_motor1_input1, LOW);
        digitalWrite(m_motor1_input2, HIGH);
        m_appliedPWMRight =
            -mapPwm(mapPWM, 1, m_MAX_PWM, m_deadbandRight, m_MAX_PWM);
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
        m_appliedPWMLeft =
            mapPwm(pwmMag, 1, m_MAX_PWM, m_deadbandLeft, m_MAX_PWM);
    }

    else if (m_requestedPWMLeft < 0) {
        digitalWrite(m_motor2_input1, LOW);
        digitalWrite(m_motor2_input2, HIGH);
        m_appliedPWMLeft =
            -mapPwm(pwmMag, 1, m_MAX_PWM, m_deadbandLeft, m_MAX_PWM);
    }
    analogWrite(enB, abs(m_appliedPWMLeft));
}

void MotorDriver::stopLeft() {
    m_appliedPWMLeft = 0;
    digitalWrite(m_motor2_input1, LOW);
    digitalWrite(m_motor2_input2, LOW);
    analogWrite(enA, m_appliedPWMLeft);
}
void MotorDriver::stopRight() {
    m_appliedPWMRight = 0;
    digitalWrite(m_motor1_input1, LOW);
    digitalWrite(m_motor1_input2, LOW);
    analogWrite(enB, m_appliedPWMRight);
}