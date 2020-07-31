#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <cstdint>
#include <tuple>
#include <vector>

#include "trajectory_generator.h"
#include "json.hpp"
#include "velocity_controller.h"

enum class VehicleState : uint8_t
{
    KL,
    LCL,
    LCR
};

struct DetectedVehicle
{
    uint32_t id;
    double x;
    double y;
    double x_vel;
    double y_vel;
    double s;
    double d;
};

class VehicleController
{
public:
    using json = nlohmann::json;

    VehicleController(const Map& map);
    Trajectory UpdatePath(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end);

private:
    static constexpr int32_t kNumOfLanes = 3;
    static constexpr double kSpeedLimit = 49.5;
    static constexpr double kBufferDistance = 30.0;
    static constexpr double kLaneWidth = 4.0;
    static constexpr double kSafeSpeed = 40.0;

    void SelectManeuver(const Vehicle& vehicle);
    void CheckSurroundings(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end);
    DetectedVehicle CurrentLaneOccuied(const Vehicle& vehicle);
    bool IsLaneChangedCompleted(const Vehicle& vehicle);

    // State specific functions
    void HandleKLState(const Vehicle& vehicle);
    void HandleLCLState(const Vehicle& vehicle);
    void HandleLCRState(const Vehicle& vehicle);

    double GetDistanceToObstacle(const Vehicle& vehicle, const DetectedVehicle& detection);
    bool IsLaneChangePossible(const Vehicle& vehicle, int32_t target_lane);

    double Sigmoid(const double& x, const double& y);
    double LaneCostFunction(const Vehicle& vehicle, int32_t target_lane);

    VehicleState state_;
    int32_t lane_;
    TrajectoryGenerator trajectory_generator_;
    VelocityController velocity_controller_;

    std::array<std::vector<DetectedVehicle>, kNumOfLanes> lanes_occupancy_;
};

#endif // VEHICLE_CONTROLLER_H
