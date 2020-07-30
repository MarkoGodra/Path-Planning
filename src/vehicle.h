#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

struct Vehicle
{
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;

    template<typename T>
    static std::vector<double> JsonArrayToVector(const T& json_array);
};

template<typename T>
std::vector<double> Vehicle::JsonArrayToVector(const T& json_array)
{
    std::vector<double> res;
    for(auto i = 0; i < json_array.size(); i++)
    {
        res.push_back(json_array[i]);
    }

    return res;
}

#endif // VEHICLE_H
