#include "VelocityController.hpp"

VelocityController::VelocityController() {
    m_velocityController.setOutputLimits(-m_maxPitch, m_maxPitch);
    m_velocityController.setPI(10.0, 0.8);
}

float VelocityController::update(float speedLeft, float speedRight,
                                 float wheelRad, float desiredVelocity,
                                 float dt) {
    m_estimatedVelocity = -wheelRad * ((speedLeft + speedRight) / 2);
    m_velocityError = desiredVelocity - m_estimatedVelocity;
    m_controllerOutput = m_velocityController.updatePI(m_velocityError, dt);
    return m_controllerOutput;
}