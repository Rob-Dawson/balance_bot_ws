#pragma once
#include "PIDController.hpp"
class VelocityController {
public:
    VelocityController();
    float update(float speedLeft, float speedRight, float wheelRad,
                 float desiredVelocity, float dt);

private:
    float m_targetSpeed;
    float m_estimatedVelocity{};
    float m_velocityError{};
    float m_desiredVelocity{};
    float m_controllerOutput{};
    // Left as non const for online tuning and adapting

    // NOLINTBEGIN(readability-magic-numbers)
    float Kp{10.0};
    float Ki{0.8};
    // NOLINTEND(readability-magic-numbers)

    PIDController m_velocityController;

    static constexpr float m_maxPitch{0.349066f};
};