#include "Encoder.hpp"
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

    PIDController m_velocityController{};

    static constexpr float m_maxPitch{0.349066f};
};