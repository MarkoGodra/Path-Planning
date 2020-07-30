#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <cstdint>
#include <tuple>
#include <vector>

#include "trajectory_generator.h"
#include "json.hpp"

enum class VehicleState : uint8_t
{
    INIT,
    KL,
    LCL,
    LCR
};

class VehicleController
{
public:
    static constexpr double kSpeedLimit = 49.5;
    static constexpr double kBufferDistance = 30.0;

    using json = nlohmann::json;

    VehicleController(const Map& map);
    Trajectory UpdatePath(Vehicle& vehicle, json::iterator& predictions_begin, json::iterator predictions_end);

private:
    VehicleState state_;
    int32_t lane_;
    double ref_vel_;
    TrajectoryGenerator trajectory_generator_;
};

#endif // VEHICLE_CONTROLLER_H