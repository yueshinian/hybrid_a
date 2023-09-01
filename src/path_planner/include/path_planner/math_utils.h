#pragma once

#include<iostream>
#include<cmath>
#include<map>

namespace common{
namespace math{

double NormalizeAngle(const double angle)
{
    double res_angle = std::fmod(angle + M_PI, 2 * M_PI);
    if(res_angle < 0){
        res_angle += 2*M_PI;
    }
    return res_angle + M_PI;
}

std::pair<double, double> Cartesian2Polar(const double x, const double y)
{
    double r = std::hypot(x, y);
    double theta = std::atan2(y, x);
    return std::make_pair(r, theta);
}

}
}