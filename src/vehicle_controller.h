#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <cstdint>
#include <tuple>
#include <vector>

#include "trajectory_generator.h"
#include "json.hpp"

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

    void SelectManeuver(const Vehicle& vehicle);
    void CheckSurroundings(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end);
    bool CurrentLaneOccuied(const Vehicle& vehicle);
    bool IsLaneChangedCompleted(const Vehicle& vehicle);

    // State specific functions
    void HandleKLState(const Vehicle& vehicle);
    void HandleLCLState(const Vehicle& vehicle);
    void HandleLCRState(const Vehicle& vehicle);

    VehicleState state_;
    int32_t lane_;
    double ref_vel_;
    TrajectoryGenerator trajectory_generator_;

    std::array<std::vector<DetectedVehicle>, kNumOfLanes> lanes_occupancy_;
};

#endif // VEHICLE_CONTROLLER_H
