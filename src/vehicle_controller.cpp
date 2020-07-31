#include "vehicle_controller.h"

#include <iostream>

VehicleController::VehicleController(const Map& map) :
    state_(VehicleState::KL),
    lane_(1.0),
    ref_vel_(0.0),
    trajectory_generator_(map),
    lanes_occupancy_()
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

    CheckSurroundings(vehicle, predictions_begin, predictions_end);
    SelectManeuver(vehicle);

    return trajectory_generator_.GenerateTrajectory(vehicle, lane_, ref_vel_);
}

void VehicleController::CheckSurroundings(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end)
{
    for(auto i = 0; i < kNumOfLanes; i++)
    {
        lanes_occupancy_[i].clear();
    }

    // Check detections for car in our lane
    for(auto& it = predictions_begin; it != predictions_end; it++)
    {
        DetectedVehicle detected_vehicle{
            (*it)[0],
                    (*it)[1],
                    (*it)[2],
                    (*it)[3],
                    (*it)[4],
                    (*it)[5],
                    (*it)[6],
        };

        // Check if detection is in region of interest
        if(!((detected_vehicle.s < vehicle.car_s + kBufferDistance) && (detected_vehicle.s > vehicle.car_s - kBufferDistance)))
        {
            continue;
        }

        // Find out detected car lane
        int detected_car_lane = 0;
        if(detected_vehicle.d >= 0 && detected_vehicle.d < kLaneWidth)
        {
            detected_car_lane = 0;
        }
        else if(detected_vehicle.d >= kLaneWidth && detected_vehicle.d < 2 * kLaneWidth)
        {
            detected_car_lane = 1;
        }
        else if(detected_vehicle.d >= 2 * kLaneWidth && detected_vehicle.d <= 3 * kLaneWidth)
        {
            detected_car_lane = 2;
        }
        else
        {
            // If vehicle is not in specified boundaries, then just skip
            continue;
        }

        lanes_occupancy_[detected_car_lane].push_back(detected_vehicle);
    }
}

void VehicleController::SelectManeuver(const Vehicle& vehicle)
{
    switch (state_) {
    case VehicleState::KL:
        // Keep lane state
        HandleKLState(vehicle);
        break;

    case VehicleState::LCL:
        // LCL state
        HandleLCLState(vehicle);
        break;

    case VehicleState::LCR:
        // LCR state
        HandleLCRState(vehicle);

        break;
    }
}

bool VehicleController::CurrentLaneOccuied(const Vehicle& vehicle)
{
    if(lanes_occupancy_[lane_].size() == 0)
    {
        return false;
    }
    else
    {
        bool current_lane_occupied = false;
        for(const auto& detection : lanes_occupancy_[lane_])
        {
            double vx = detection.x_vel;
            double vy = detection.y_vel;

            double check_car_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = detection.s;

            check_car_s += 0.02 * check_car_speed;

            // detection in front
            current_lane_occupied |= (check_car_s > vehicle.car_s) && ((check_car_s - vehicle.car_s) <= kBufferDistance);
        }

        return current_lane_occupied;
    }
}

void VehicleController::HandleKLState(const Vehicle& vehicle)
{
    bool current_lane_occupied = CurrentLaneOccuied(vehicle);

    if(current_lane_occupied)
    {
        std::cout << "Occupied" << std::endl;
        ref_vel_ -= 0.224;
    }
    else
    {
        // If lane is not occupied, stay in the same lane and don't accelerate if possible
        // Lane keeping, drive the speed limit
        if(ref_vel_ < kSpeedLimit)
        {
            ref_vel_ += 0.224;
        }
    }

    // TODO: Analyze potential overtakes
    if(current_lane_occupied)
    {
        bool left_turn_possible = false;
        bool right_turn_possible = false;

        if(lane_ > 0)
        {
            left_turn_possible = true;
        }

        if(lane_ < 2)
        {
            right_turn_possible = true;
        }

        if(left_turn_possible)
        {
            int32_t potential_lane = lane_ - 1;

            // If left lane is unoccupied
            if(lanes_occupancy_[potential_lane].size() == 0)
            {
                // State transition to LCL
                std::cout << "Transition to LCL" << std::endl;
                lane_--;
                state_ = VehicleState::LCL;
            }
        }
        else if(right_turn_possible)
        {
            int32_t potential_lane = lane_ + 1;

            // If left lane is unoccupied
            if(lanes_occupancy_[potential_lane].size() == 0)
            {
                // State transition to LCL
                std::cout << "Transition to LCL" << std::endl;
                lane_++;
                state_ = VehicleState::LCR;
            }
        }
    }
}

void VehicleController::HandleLCLState(const Vehicle& vehicle)
{
    // TODO: Check for front collision and slow down if needed

    if(IsLaneChangedCompleted(vehicle))
    {
        std::cout << "Transition to KL" << std::endl;
        state_ = VehicleState::KL;
    }
    else
    {
        std::cout << "Changing lane" << std::endl;
    }
}

void VehicleController::HandleLCRState(const Vehicle& vehicle)
{
    // TODO: Check for front collision and slow down if needed

    if(IsLaneChangedCompleted(vehicle))
    {
        std::cout << "Transition to KL" << std::endl;
        state_ = VehicleState::KL;
    }
    else
    {
        std::cout << "Changing lane" << std::endl;
    }
}

bool VehicleController::IsLaneChangedCompleted(const Vehicle& vehicle)
{
    double target_lane_center = lane_ * kLaneWidth + (kLaneWidth / 2);
    if((vehicle.car_d >= target_lane_center - kLaneWidth / 5.0) && (vehicle.car_d <= target_lane_center + kLaneWidth / 5.0))
    {
        return true;
    }
    else
    {
        return false;
    }
}
