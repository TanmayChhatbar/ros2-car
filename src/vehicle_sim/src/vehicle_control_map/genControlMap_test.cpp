#include "Vehicle2D.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

double cost_function(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
    // inputs
    // - rear wheel speed (assumed same)
    // - steering angle [rad]
    // output
    // - score based on velocity derivatives

    // load data
    Vehicle2D &vehicle = *static_cast<Vehicle2D *>(f_data);
    Vehicle2DData &data = vehicle.getVehicle2DData();
    Vehicle2DConfig &config = vehicle.getVehicle2DConfig();

    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);

    // set inputs
    double w_wheel[4];
    if (config.getDrivetrainType() == DrivetrainType_E::RWD)
    {
        w_wheel[0] = w_wheel_eq[0];
        w_wheel[1] = w_wheel_eq[1];
        w_wheel[2] = x[0];
        w_wheel[3] = x[0]; // only rear wheels are driven
    }
    else if (config.getDrivetrainType() == DrivetrainType_E::FWD)
    {
        w_wheel[0] = x[0];
        w_wheel[1] = x[0];
        w_wheel[2] = w_wheel_eq[2];
        w_wheel[3] = w_wheel_eq[3]; // only front wheels are driven
    }
    else if (config.getDrivetrainType() == DrivetrainType_E::AWD)
    {
        w_wheel[0] = x[0];
        w_wheel[1] = x[0];
        w_wheel[2] = x[0];
        w_wheel[3] = x[0]; // all wheels are driven
    }
    else
    {
        std::cerr << "Unknown drivetrain type!" << std::endl;
        return 1e6;
    }
    data.setWheelVelocities(w_wheel);
    // data.setSteeringAngle(u[1]);

    // calculate and get accelerations
    vehicle.calcTireNormalLoads();
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcNetForcesAndMoments();
    vehicle.calcBodyAccelerations();
    vehicle.calcWheelAccelerations();
    double ax, ay, a_yaw;
    double a_wheel[4];
    data.getLinearAccelerations(ax, ay);
    data.getAngularAccelerations(a_yaw);
    data.getWheelAccelerations(a_wheel);

    // calculate score
    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    double ax_true = ax + w_yaw * vy; // actual rate of change of vx
    double ay_true = ay - w_yaw * vx; // actual rate of change of vy

    double score = ax_true * ax_true + ay_true * ay_true + a_yaw * a_yaw;

    (void)grad; // unused
    return score;
}

void stabilizeWheelSpeeds(Vehicle2D &vehicle, double vx_target, double vy_target, double w_yaw_target)
{
    // stabilize wheel speeds (to solve equilibrium for non-driven wheels)
    Vehicle2DData &data = vehicle.getVehicle2DData();

    double ax = -w_yaw_target * vy_target; // [m/s^2] ax_true = 0 = ax + w_yaw * vy
    double ay = w_yaw_target * vx_target;  // [m/s^2] ay_true = 0 = ay - w_yaw * vx
    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(0.0); // [rad/s^2]
    data.setLinearVelocities(vx_target, vy_target);
    data.setAngularVelocities(w_yaw_target);

    double max_a_wheel = 10.0;
    while (max_a_wheel > 0.01)
    {
        // override wheel torques to zero
        double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
        data.setWheelTorques(wheel_torques);
        vehicle.calcTireNormalLoads();
        vehicle.calcWheelSlipsAndForces();
        vehicle.calcNetForcesAndMoments();
        vehicle.calcWheelAccelerations();
        double a_wheel[4];
        double w_wheel[4];
        data.getWheelAccelerations(a_wheel);
        for (int i = 0; i < 4; ++i)
        {
            w_wheel[i] += a_wheel[i] * 0.0001;
        }
        data.setWheelVelocities(w_wheel);
        max_a_wheel = std::max({std::abs(a_wheel[0]), std::abs(a_wheel[1]), std::abs(a_wheel[2]), std::abs(a_wheel[3])});
    }
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);
    std::cout << "free-rolling wheel speeds:\n\t"
              << w_wheel_eq[0] << "\t"
              << w_wheel_eq[1] << "\n\t"
              << w_wheel_eq[2] << "\t"
              << w_wheel_eq[3] << std::endl;
}

bool optimize(Vehicle2D &vehicle, double vx_target, double vy_target, double w_yaw_target)
{
    stabilizeWheelSpeeds(vehicle, vx_target, vy_target, w_yaw_target);

    Vehicle2DData &data = vehicle.getVehicle2DData();

    // get wheel speeds when free-rolling
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);

    // write data that wont change
    double ax = -w_yaw_target * vy_target; // [m/s^2] ax_true = 0 = ax + w_yaw * vy
    double ay = w_yaw_target * vx_target;  // [m/s^2] ay_true = 0 = ay - w_yaw * vx
    data.setLinearVelocities(vx_target, vy_target);
    data.setAngularVelocities(w_yaw_target);
    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(0.0);

    // optimization
    nlopt::opt opt(nlopt::GN_DIRECT_L, 2);
    opt.set_min_objective(cost_function, &vehicle);
    std::vector<double> lb = {0.0, DEG2RAD(0.0)};                             // lower bounds for [rear wheel speed, steering angle]
    std::vector<double> ub = {(w_wheel_eq[2] + w_wheel_eq[3]), DEG2RAD(0.0)}; // upper bounds for [rear wheel speed, steering angle]
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-5);
    opt.set_ftol_abs(1e-5);
    opt.set_stopval(1e-6);
    std::vector<double> x = {w_wheel_eq[2], 0.0};
    double minf;
    try
    {
        (void)opt.optimize(x, minf);
    }
    catch (std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
        return false;
    }
    return true;
}

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../../configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);
    Vehicle2DData &data = vehicle.getVehicle2DData();

    // optimize throttle to maintain speed
    double vx_target = 5.0;   // [m/s]
    double vy_target = 0.0;    // [m/s]
    double w_yaw_target = 0.0; // [rad/s]

    // stabilize wheel speeds (useful to solve equilibrium for non-driven wheels)
    if (optimize(vehicle, vx_target, vy_target, w_yaw_target))
    {
        // get optimal values
        double optimalWheelSpeeds[4];
        data.getWheelVelocities(optimalWheelSpeeds);
        double optimalSteeringAngle = data.getSteeringAngle();

        std::cout << "optimal wheel speeds:\n\t"
                  << optimalWheelSpeeds[0] << "\t"
                  << optimalWheelSpeeds[1] << "\n\t"
                  << optimalWheelSpeeds[2] << "\t"
                  << optimalWheelSpeeds[3] << std::endl;
        std::cout << "optimal steering angle: " << optimalSteeringAngle * 180.0 / M_PI << " [deg]\n";
    }

    // print wheel speeds

    // get traction torques

    // calculate motor torque

    // calculate throttle input command
}
