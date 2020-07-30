#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(const Map& map) :
    map_(map)
{

}

Trajectory TrajectoryGenerator::GenerateTrajectory(const Vehicle& vehicle, int32_t target_lane, double target_vel)
{
    // Size of previous path
    int prev_size = vehicle.previous_path_x.size();

    // Path generation
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    double ref_x = vehicle.car_x;
    double ref_y = vehicle.car_y;
    double ref_yaw = deg2rad(vehicle.car_yaw);

    if(prev_size < 2)
    {
        // Initial case
        double prev_car_x = vehicle.car_x - cos(vehicle.car_yaw);
        double prev_car_y = vehicle.car_y - sin(vehicle.car_yaw);

        // Use car current location and previous reconstructed from current yaw

        ptsx.push_back(prev_car_x);
        ptsx.push_back(vehicle.car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(vehicle.car_y);
    }
    else
    {
        // Normal case when we have previous
        // Take two latest values
        ref_x = vehicle.previous_path_x[prev_size - 1];
        ref_y = vehicle.previous_path_y[prev_size - 1];

        double ref_x_prev = vehicle.previous_path_x[prev_size - 2];
        double ref_y_prev = vehicle.previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }


    // Get anchors for spline
    // 3 points 30 meters apart
    for(auto i = 1; i <= 3; i++)
    {
        double s_increment = i * 30;
        vector<double> xy = getXY(vehicle.car_s + s_increment, (2 + 4 * target_lane), map_.map_waypoints_s, map_.map_waypoints_x, map_.map_waypoints_y);

        // Now we are generating path in front
        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);
    }

    // Transformation - Go from global coordiantes to car local frame
    for(auto i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create spline
    tk::spline spline;

    // Set anchor points for spline
    spline.set_points(ptsx, ptsy);

    // Now that we have spline all ready, we need to calculate filler points
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    // Take uneaten points from previous path
    for (auto i = 0; i < vehicle.previous_path_x.size(); i++)
    {
        next_x_vals.push_back(vehicle.previous_path_x[i]);
        next_y_vals.push_back(vehicle.previous_path_y[i]);
    }

    // Horizon
    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dst = sqrt((target_x * target_x) + (target_y * target_y));

    // Use this as our current position on x axis for generation of x axis values (increments)
    double x_add_on = 0.0;


    for(auto i = 1; i <= 50 - vehicle.previous_path_x.size(); i++)
    {
        double N = target_dst / (0.02 * target_vel / 2.24);
        double x_point = x_add_on + (target_x / N);
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Transform back to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return Trajectory{next_x_vals, next_y_vals};
}
