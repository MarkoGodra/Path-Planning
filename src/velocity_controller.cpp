#include "velocity_controller.h"


VelocityController::VelocityController() :
    target_velocity_(kMaximumVelocity),
    current_velocity_(0.0),
    state_(VelocityAction::kAccelerate)
{

}

void VelocityController::SetTargetVelocity(double target_velocity)
{
    target_velocity_ = target_velocity;
}

double VelocityController::CalculateVelocity()
{
    if(state_ == VelocityAction::kAccelerate)
    {
        if(current_velocity_ < target_velocity_)
        {
            current_velocity_ += kMaximumAcceleration;
        }
    }
    else if(state_ == VelocityAction::kSlowdown)
    {
        // If lane is not occupied, stay in the same lane and don't accelerate if possible
        // Lane keeping, drive the speed limit
        if(current_velocity_ > kMaximumAcceleration)
        {
            current_velocity_ -= kMaximumAcceleration;
        }
    }
    else if(state_ == VelocityAction::kConstantSpeed)
    {
        if(current_velocity_ > target_velocity_)
        {
            current_velocity_ -= kMaximumAcceleration;
        }
        else if(current_velocity_ < target_velocity_)
        {
            current_velocity_ += kMaximumAcceleration;
        }
    }

    return current_velocity_;
}

double VelocityController::SetAction(const VelocityAction& action)
{
    state_ = action;
}
