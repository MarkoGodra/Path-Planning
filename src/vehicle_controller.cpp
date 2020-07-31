#include "vehicle_controller.h"

#include <iostream>

VehicleController::VehicleController(const Map& map) :
    state_(VehicleState::KL),
    lane_(1.0),
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

    return trajectory_generator_.GenerateTrajectory(vehicle, lane_, velocity_controller_.CalculateVelocity());
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
        if(!((detected_vehicle.s < vehicle.car_s + 2 * kBufferDistance) && (detected_vehicle.s > vehicle.car_s - kBufferDistance)))
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

DetectedVehicle VehicleController::CurrentLaneOccuied(const Vehicle& vehicle)
{
    if(lanes_occupancy_[lane_].size() == 0)
    {
        DetectedVehicle ret;
        ret.id = std::numeric_limits<uint32_t>::max();
        return ret;
    }
    else
    {
        DetectedVehicle vehicle_in_front;
        vehicle_in_front.s = std::numeric_limits<double>::max();
        for(const auto& detection : lanes_occupancy_[lane_])
        {
            // If car is in front and distance is shorter than previous short
            if(detection.s > vehicle.car_s && vehicle_in_front.s > detection.s)
            {
                vehicle_in_front = detection;
            }
        }

        return vehicle_in_front;
    }
}

void VehicleController::HandleKLState(const Vehicle& vehicle)
{
    DetectedVehicle vehicle_in_front = CurrentLaneOccuied(vehicle);
    bool should_overtake = false;

    // If front vehicle detection is valid
    if(vehicle_in_front.id != std::numeric_limits<uint32_t>::max() && vehicle_in_front.s != std::numeric_limits<double>::max())
    {
        double distance_to_vehicle = GetDistanceToObstacle(vehicle, vehicle_in_front);

        // In order to avoid chrash brake
        if(distance_to_vehicle <= kBufferDistance * 0.25)
        {
            velocity_controller_.SetAction(VelocityAction::kSlowdown);
        }
        if(distance_to_vehicle <= kBufferDistance / 2 && distance_to_vehicle > kBufferDistance * 0.25)
        {
            // Take speed of vehicle in front
            double target_speed = sqrt(vehicle_in_front.x_vel * vehicle_in_front.x_vel + vehicle_in_front.y_vel * vehicle_in_front.y_vel);
            velocity_controller_.SetTargetVelocity(target_speed);
            velocity_controller_.SetAction(VelocityAction::kConstantSpeed);
        }
        else if(distance_to_vehicle < kBufferDistance * 0.75 && distance_to_vehicle > kBufferDistance / 2)
        {
            // If distance is not too close, just keep constant velocity
            velocity_controller_.SetTargetVelocity(kSafeSpeed);
            velocity_controller_.SetAction(VelocityAction::kConstantSpeed);
        }

        // Mark that we should execute maneuver if our lane is taken
        should_overtake = true;
    }
    else
    {
        velocity_controller_.SetTargetVelocity(kSpeedLimit);
        velocity_controller_.SetAction(VelocityAction::kAccelerate);
        should_overtake = false;
    }

    if(should_overtake)
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

        double left_turn_cost = 2.0;
        if(left_turn_possible)
        {
            int32_t potential_lane = lane_ - 1;

            // If left lane is unoccupied
            if(IsLaneChangePossible(vehicle, potential_lane))
            {
                left_turn_cost = LaneCostFunction(vehicle, potential_lane);
            }
        }

        double right_turn_cost = 2.0;
        if(right_turn_possible)
        {
            int32_t potential_lane = lane_ + 1;

            // If left lane is unoccupied
            if(IsLaneChangePossible(vehicle, potential_lane))
            {
                right_turn_cost = LaneCostFunction(vehicle, potential_lane);
            }
        }

        if(right_turn_cost < 2.0 || left_turn_cost < 2.0)
        {
            if(left_turn_cost <= right_turn_cost)
            {
                // Turn left
                // State transition to LCL
                velocity_controller_.SetTargetVelocity(kSpeedLimit);
                velocity_controller_.SetAction(VelocityAction::kAccelerate);
                lane_--;
                state_ = VehicleState::LCL;
            }
            else if(left_turn_cost > right_turn_cost)
            {
                // Turn right
                // State transition to LCR
                velocity_controller_.SetTargetVelocity(kSpeedLimit);
                velocity_controller_.SetAction(VelocityAction::kAccelerate);
                lane_++;
                state_ = VehicleState::LCR;
            }
        }
    }
}

void VehicleController::HandleLCLState(const Vehicle& vehicle)
{
    if(IsLaneChangedCompleted(vehicle))
    {
        state_ = VehicleState::KL;
    }
}

void VehicleController::HandleLCRState(const Vehicle& vehicle)
{
    if(IsLaneChangedCompleted(vehicle))
    {
        state_ = VehicleState::KL;
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

double VehicleController::GetDistanceToObstacle(const Vehicle& vehicle, const DetectedVehicle& detection)
{
    double vx = detection.x_vel;
    double vy = detection.y_vel;

    double check_car_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = detection.s;

    check_car_s += 0.02 * check_car_speed;

    return check_car_s - vehicle.car_s;
}

bool VehicleController::IsLaneChangePossible(const Vehicle& vehicle, int32_t target_lane)
{
    if(lanes_occupancy_[target_lane].size() == 0)
    {
        return true;
    }
    else
    {
        bool gap_available = true;
        for(const auto& detection : lanes_occupancy_[target_lane])
        {
            double vx = detection.x_vel;
            double vy = detection.y_vel;

            double velocity = sqrt((vx * vx) + (vy * vy));
            double detection_s_position = detection.s + 0.02 * velocity;

            if((detection_s_position < vehicle.car_s + kBufferDistance * 0.75) && (detection_s_position > vehicle.car_s - kBufferDistance * 0.75))
            {
                gap_available = false;
            }
        }

        return gap_available;
    }
}

// Cost function should be bigger if more cars are detected in target lane
// Cost function should be bigger when distance to the nearest car is short
// Cost function should be bigger the more difference between closest car speed to the speed limit
double VehicleController::Sigmoid(const double& x, const double& y)
{
    double y_ = y;
    if(y < 0.001 && y > -0.001)
    {
        y_ = 0.0001;
    }

    return 1 - exp((-x) / y_);
}

double VehicleController::LaneCostFunction(const Vehicle& vehicle, int32_t target_lane)
{
    if(lanes_occupancy_[target_lane].size() == 0)
    {
        return 0.0;
    }

    double occupancy_cost = Sigmoid(1, lanes_occupancy_[target_lane].size());

    DetectedVehicle nearest_vehicle = lanes_occupancy_[target_lane][0];
    for(auto i = 1; i < lanes_occupancy_[target_lane].size(); i++)
    {
        if(lanes_occupancy_[target_lane][i].s > vehicle.car_s && lanes_occupancy_[target_lane][i].s < nearest_vehicle.s)
        {
            nearest_vehicle = lanes_occupancy_[target_lane][i];
        }
    }

    double distance_cost = Sigmoid(1.0, nearest_vehicle.s - vehicle.car_s);

    double nearest_velocity = sqrt(nearest_vehicle.x_vel * nearest_vehicle.x_vel + nearest_vehicle.y_vel * nearest_vehicle.y_vel);
    double velocity_cost = Sigmoid(nearest_velocity / kSpeedLimit, 1.0);

    double final_weight = 2 * occupancy_cost + 3 * distance_cost + 3 * velocity_cost;

    return Sigmoid(final_weight, 1.0);
}
