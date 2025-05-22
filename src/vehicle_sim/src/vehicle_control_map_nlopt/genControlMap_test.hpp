#include "Vehicle2D.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#define DEBUG_STABILIZEWHEELSPEEDS 0
#define DEBUG_OPTIMIZATION 0

#define DEG2RAD(x) ((x) * M_PI / 180.0)

void stabilizeWheelSpeeds(Vehicle2D &vehicle)
{
    // stabilize wheel speeds (to solve equilibrium for non-driven wheels)
    // get data
    Vehicle2DData &data = vehicle.getVehicle2DData();
    Vehicle2DConfig &config = vehicle.getVehicle2DConfig();

    // calculate approximate wheel speeds
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double half_track_width = config.getTrackWidth() / 2.0;
    const double r_wheel = config.getWheelRadius();
    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    const double vx_yaw = half_track_width * w_yaw;
    double vxw[4] = {vx - vx_yaw, vx + vx_yaw,
                     vx - vx_yaw, vx + vx_yaw};
    double vyw[4] = {vy + a * w_yaw, vy + a * w_yaw,
                     vy - b * w_yaw, vy - b * w_yaw};
    double w_wheel[4];
    for (int i = 0; i < 2; ++i)
    {
        vxw[i] = vxw[i] * std::cos(steering_angle) + vyw[i] * std::sin(steering_angle);
        w_wheel[i] = vxw[i] / r_wheel;
    }
    for (int i = 2; i < 4; ++i)
    {
        w_wheel[i] = vxw[i] / r_wheel;
    }
        
    // set data
    const double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    data.setWheelTorques(wheel_torques);
    data.setWheelVelocities(w_wheel);

    // evolve wheel speeds until equilibrium
    uint iter = 0;
    double a_wheel[4];
    double max_a_wheel = 1.0;
    while (max_a_wheel > 1.0e-5 && iter < 200)
    {
        iter++;
        vehicle.calcWheelSlipsAndForces();
        vehicle.calcNetForcesAndMoments();
        vehicle.calcWheelAccelerations();
        data.getWheelAccelerations(a_wheel);
        for (int i = 0; i < 4; ++i)
        {
            w_wheel[i] += a_wheel[i] * 0.0002;
        }
        data.setWheelVelocities(w_wheel);
        max_a_wheel = std::max({std::abs(a_wheel[0]),
                                std::abs(a_wheel[1]),
                                std::abs(a_wheel[2]),
                                std::abs(a_wheel[3])});
    }
#if DEBUG_STABILIZEWHEELSPEEDS
    std::cout << "iter = " << iter << "\n";
    std::cout << "stabilized wheel speeds:\n\t";
    std::cout << w_wheel[0] << "\t"
              << w_wheel[1] << "\n\t"
              << w_wheel[2] << "\t"
              << w_wheel[3] << std::endl;
#endif
}
