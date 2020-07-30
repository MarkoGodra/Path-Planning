#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "helpers.h"
#include "spline.h"
#include "vehicle.h"

#include <tuple>
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
    std::tuple<std::vector<double>, std::vector<double>> GenerateAnchorPoints(const Vehicle& vehicle, double forward_anchors_distance,
                                                                              int32_t target_lane, double& ref_x, double& ref_y, double& ref_yaw);
    Trajectory GenerateFillerPoints(const Vehicle& vehicle,
                                    const vector<double>& anchors_x,
                                    const vector<double>& anchors_y,
                                    const double& target_vel,
                                    const double& ref_x,
                                    const double& ref_y,
                                    const double& ref_yaw);
    void GCoordSysToLocalCoord(std::vector<double>& ptsx, std::vector<double>& ptsy,
                               const double& ref_x, const double& ref_y, const double& ref_yaw);
    const Map map_;
    static constexpr double kHorizon = 30.0;
    static constexpr int32_t kNumOfAnchors = 3;
    static constexpr double kForwardAnchorsDistance = 30.0;
    static constexpr double kLaneWidth = 4.0;
};

#endif // TRAJECTORY_GENERATOR_H
