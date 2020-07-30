#include "vehicle_controller.h"

VehicleController::VehicleController(const Map& map) :
    state_(VehicleState::INIT),
    lane_(1.0),
    ref_vel_(0.0),
    trajectory_generator_(map)
{

}

Trajectory VehicleController::UpdatePath(Vehicle& vehicle,
                                         json::iterator& predictions_begin,
                                         json::iterator predictions_end)
{
    // Size of previous path
    int prev_size = vehicle.previous_path_x.size();

    // If it is not begining, s is last consumed value from previous path
    if(prev_size > 0)
    {
        vehicle.car_s = vehicle.end_path_s;
    }

    // There is no collision warning on each start of each iteration
    bool collision_warning = false;

    // Check detections for car in our lane
    for(auto& it = predictions_begin; it != predictions_end; it++)
    {
        // Detected car d coordinate
        double d = (*it)[6];

        // If car is in our lane
        if(d < (2 + 4 * lane_ + 2) && d > (2 + 4 * lane_ - 2))
        {
            double vx = (*it)[3];
            double vy = (*it)[4];

            double check_car_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = (*it)[5];

            check_car_s += static_cast<double>(prev_size) * 0.02 * check_car_speed;
            if((check_car_s > vehicle.car_s) && (check_car_s - vehicle.car_s < 30))
            {
                collision_warning = true;
            }
        }
    }

    if(collision_warning)
    {
        ref_vel_ -= 0.224;
    }
    else
    {
        if(ref_vel_ < kSpeedLimit)
        {
            ref_vel_ += 0.224;
        }
    }

    return trajectory_generator_.GenerateTrajectory(vehicle, lane_, ref_vel_);
}
