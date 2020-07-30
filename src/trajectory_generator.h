#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "helpers.h"
#include "spline.h"
#include "vehicle.h"

#include <vector>

struct Trajectory
{
    std::vector<double> path_x;
    std::vector<double> path_y;
};

struct Map
{
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const Map& map);

    Trajectory GenerateTrajectory(const Vehicle& vehicle, int32_t target_lane, double target_vel);

private:
    const Map map_;
};

#endif // TRAJECTORY_GENERATOR_H
