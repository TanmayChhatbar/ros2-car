#include "Vehicle2D.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

void stabilizeWheelSpeeds(Vehicle2D &vehicle)
{
    // stabilize wheel speeds (to solve equilibrium for non-driven wheels)
    Vehicle2DData &data = vehicle.getVehicle2DData();
    double max_a_wheel = 10.0;
    double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    data.setWheelTorques(wheel_torques);

    uint iter = 0;
    while (max_a_wheel > 1.0e-5 && iter < 100000)
    {
        iter++;
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
        max_a_wheel = std::max({std::abs(a_wheel[0]),
                                std::abs(a_wheel[1]),
                                std::abs(a_wheel[2]),
                                std::abs(a_wheel[3])});
    }
}

double cost_function(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
    // inputs
    // - rear wheel speed (assumed same)
    // - steering angle [rad]
    // - yaw rate [rad/s]
    // output
    // - score based on velocity derivatives

    // load data
    Vehicle2D &vehicle = *static_cast<Vehicle2D *>(f_data);
    Vehicle2DData &data = vehicle.getVehicle2DData();
    Vehicle2DConfig &config = vehicle.getVehicle2DConfig();

    // get target data
    double vx_target, vy_target;
    data.getLinearVelocities(vx_target, vy_target);
    double w_yaw_target = x[2];

    // write inputs to vehicle data
    double ax = -w_yaw_target * vy_target; // ax_true = 0 = ax + w_yaw * vy
    double ay = w_yaw_target * vx_target;  // ay_true = 0 = ay - w_yaw * vx
    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(0.0);
    data.setAngularVelocities(w_yaw_target);
    data.setSteeringAngle(x[1]);

    // stabilize wheel speeds to solve equilibrium for non-driven wheels
    stabilizeWheelSpeeds(vehicle);
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

    // calculate body accelerations
    vehicle.calcTireNormalLoads();
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcNetForcesAndMoments();
    vehicle.calcBodyAccelerations();

    double ax_calc, ay_calc, a_yaw_calc;
    data.getLinearAccelerations(ax_calc, ay_calc);
    data.getAngularAccelerations(a_yaw_calc);

    // calculate score
    double ax_true = ax_calc + w_yaw_target * vy_target; // actual rate of change of vx
    double ay_true = ay_calc - w_yaw_target * vx_target; // actual rate of change of vy

    double score = ax_true * ax_true + ay_true * ay_true + a_yaw_calc * a_yaw_calc;
    (void)grad; // unused
    return score;
}

bool optimize(Vehicle2D &vehicle, double vx_target, double vy_target)
{
    Vehicle2DData &data = vehicle.getVehicle2DData();
    Vehicle2DConfig &config = vehicle.getVehicle2DConfig();

    data.setLinearVelocities(vx_target, vy_target);
    double wheel_speed_at_vx = vx_target / config.getWheelRadius();

    // optimization
    nlopt::opt opt(nlopt::GN_DIRECT_L, 3);
    opt.set_min_objective(cost_function, &vehicle);
    std::vector<double> lb = {
        wheel_speed_at_vx,
        DEG2RAD(-40.0),
        0.0};
    std::vector<double> ub = {
        wheel_speed_at_vx * 5.0,
        DEG2RAD(40.0),
        1.0};
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-5);
    opt.set_ftol_abs(1e-5);
    opt.set_stopval(1e-6);
    std::vector<double> x = {wheel_speed_at_vx, 0.0, 0.0};
    double minf;
    std::cout << "Starting optimization\n";
    try
    {
        (void)opt.optimize(x, minf);
        std::cout << "score: " << minf << std::endl;
        std::vector <double> grad;
        cost_function(x, grad, &vehicle);
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
    double vx_target = 1.0; // [m/s]
    double vy_target = 0.4; // [m/s]
    double v = std::sqrt(vx_target * vx_target + vy_target * vy_target);
    std::cout << "target speed: " << v << " [m/s]\n";
    std::cout << "approx wheel speed: " << vx_target / config.getWheelRadius() << " [rad/s]\n";

    // stabilize wheel speeds (useful to solve equilibrium for non-driven wheels)
    if (optimize(vehicle, vx_target, vy_target))
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
