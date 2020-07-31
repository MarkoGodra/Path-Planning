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
    bool left_lane_occupied;
    bool right_lane_occupied;
    double forward_vehicle_speed = -1.0;
    std::tuple<bool, bool, bool> lanes_occupancy = CheckSurroundings(vehicle, predictions_begin, predictions_end);
    SelectManeuver(vehicle, predictions_begin, predictions_end, lanes_occupancy);

    return trajectory_generator_.GenerateTrajectory(vehicle, lane_, ref_vel_);
}

std::tuple<bool, bool, bool> VehicleController::CheckSurroundings(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end)
{
    // There is no collision warning on each start of each iteration
    bool current_lane_occupied = false;
    bool left_lane_occupied = false;
    bool right_lane_occupied = false;

    // Check detections for car in our lane
    for(auto& it = predictions_begin; it != predictions_end; it++)
    {
        // Detected car d coordinate
        double d = (*it)[6];

        // Find out detected car lane
        int car_lane = 0;
        if(d >= 0 && d < kLaneWidth)
        {
            car_lane = 0;
        }
        else if(d >= kLaneWidth && d < 2 * kLaneWidth)
        {
            car_lane = 1;
        }
        else if(d >= 2 * kLaneWidth && d <= 3 * kLaneWidth)
        {
            car_lane = 2;
        }
        else
        {
            // If vehicle is not in specified boundaries, then just skip
            continue;
        }

        double vx = (*it)[3];
        double vy = (*it)[4];

        double check_car_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = (*it)[5];

        check_car_s += static_cast<double>(vehicle.previous_path_x.size()) * 0.02 * check_car_speed;

        // detection in front
        if(car_lane == lane_)
        {
            current_lane_occupied |= (check_car_s > vehicle.car_s) && ((check_car_s - vehicle.car_s) <= kBufferDistance);
        }
        else if(lane_ == car_lane - 1)
        {
            left_lane_occupied |= (check_car_s <= (vehicle.car_s + kBufferDistance)) && (check_car_s >= (vehicle.car_s - kBufferDistance));
        }
        else if(lane_ == car_lane + 1)
        {
            right_lane_occupied |= (check_car_s <= (vehicle.car_s + kBufferDistance)) && (check_car_s >= (vehicle.car_s - kBufferDistance));
        }
    }

    return std::tie(current_lane_occupied, left_lane_occupied, right_lane_occupied);
}

void VehicleController::SelectManeuver(const Vehicle& vehicle,
                                       json::iterator& predictions_begin,
                                       json::iterator predictions_end,
                                       const std::tuple<bool, bool, bool>& lanes_occupancy)
{
    bool current_lane_occupied = std::get<0>(lanes_occupancy);
    bool left_lane_occupied = std::get<1>(lanes_occupancy);
    bool right_lane_occupied = std::get<2>(lanes_occupancy);

    if(current_lane_occupied)
    {
        // If left lane change is possible
        if(!left_lane_occupied && lane_ > 0)
        {
            lane_--;
        }
        // If right lane change is possible
        else if(!right_lane_occupied && lane_ < 2)
        {
            lane_++;
        }
        // If both conditions fail keep lane and slow down
        else
        {
            ref_vel_ -= 0.224;
        }
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
