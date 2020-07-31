#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <cstdint>

enum class VelocityAction : uint8_t
{
    kAccelerate,
    kSlowdown,
    kConstantSpeed
};

class VelocityController
{
public:
    VelocityController();

    void SetTargetVelocity(double target_velocity);
    double CalculateVelocity();
    double SetAction(const VelocityAction& action);

private:
    static constexpr double kMaximumVelocity = 49.5;
    static constexpr double kMaximumAcceleration = 0.5;
    static constexpr double kMaximumDeacceleration = 0.224;

    double target_velocity_;
    double current_velocity_;

    VelocityAction state_;
};

#endif // VELOCITY_CONTROLLER_H
