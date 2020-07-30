#include "vehicle_controller.h"

#include <iostream>

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
    bool current_lane_occupied;
    double forward_vehicle_speed = -1.0;
    std::tie(current_lane_occupied, forward_vehicle_speed) = CheckForForwardDetection(vehicle, predictions_begin, predictions_end);
    SelectManeuver(vehicle, predictions_begin, predictions_end, current_lane_occupied, forward_vehicle_speed);

    return trajectory_generator_.GenerateTrajectory(vehicle, lane_, ref_vel_);
}

std::tuple<bool, double> VehicleController::CheckForForwardDetection(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end)
{
    // There is no collision warning on each start of each iteration
    bool current_lane_occupied = false;
    double check_car_speed;

    // Check detections for car in our lane
    for(auto& it = predictions_begin; it != predictions_end; it++)
    {
        // Detected car d coordinate
        double d = (*it)[6];

        // If car is in our lane
        if(d < ((kLaneWidth / 2) + kLaneWidth * lane_ + (kLaneWidth / 2)) && d > ((kLaneWidth / 2) + kLaneWidth * lane_ - (kLaneWidth / 2)))
        {
            double vx = (*it)[3];
            double vy = (*it)[4];

            check_car_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = (*it)[5];

            check_car_s += static_cast<double>(vehicle.previous_path_x.size()) * 0.02 * check_car_speed;
            if((check_car_s > vehicle.car_s) && (check_car_s - vehicle.car_s < kBufferDistance))
            {
                std::cout << "Occupied" << std::endl;
                current_lane_occupied = true;
            }
        }
    }

    return std::tie(current_lane_occupied, check_car_speed);
}

void VehicleController::SelectManeuver(const Vehicle& vehicle,
                                       json::iterator& predictions_begin,
                                       json::iterator predictions_end, bool current_lane_occupied, const double& car_in_front_speed)
{
    if(current_lane_occupied)
    {
//        // If left lane change is possible
//        if(LaneChangeLeftPossible())
//        {
//            if(lane_ > 0)
//            {
//                lane_--;
//            }
//        }
//        // If right lane change is possible
//        else if(LaneChangeRightPossible())
//        {
//            if(lane_ < 2)
//            {
//                lane_++;
//            }
//        }
//        // If both conditions fail keep lane and slow down
//        else
//        {
//            ref_vel_ -= car_in_front_speed * 0.8;
//        }

        ref_vel_ -= 0.224;
    }
    else
    {
        // Lane keeping, drive the speed limit
        if(ref_vel_ < kSpeedLimit)
        {
            ref_vel_ += 0.224;
        }
    }
}
