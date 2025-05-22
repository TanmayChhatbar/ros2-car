#include "Vehicle2D.hpp"
#include "OptSolver.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#define DEG2RAD(x) ((x) * M_PI / 180.0)

double cost_function(Vehicle2D &vehicle, std::vector<double> u)
{
    // cost function
    // inputs
    // - rear wheel speed (assumed same)
    // - steering angle [rad]
    // output - score based on accelerations

    Vehicle2DData &data = vehicle.getVehicle2DData();
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);

    // write inputs
    double w_wheel[4] = {w_wheel_eq[0], w_wheel_eq[1], u[0], u[0]}; // only rear wheels are driven
    data.setWheelVelocities(w_wheel);
    // data.setSteeringAngle(u[1]);

    // calculate accelerations
    vehicle.calcTireNormalLoads();
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcNetForcesAndMoments();
    vehicle.calcBodyAccelerations();
    vehicle.calcWheelAccelerations();

    // get accelerations
    double ax, ay, a_yaw;
    double a_wheel[4];
    data.getLinearAccelerations(ax, ay);
    data.getAngularAccelerations(a_yaw);
    data.getWheelAccelerations(a_wheel);

    // calculate score
    // double score = -(ax * ax + ay * ay +
    //                  a_wheel[0] * a_wheel[0] + a_wheel[1] * a_wheel[1] +
    //                  a_wheel[2] * a_wheel[2] + a_wheel[3] * a_wheel[3]);
    // double score = (ax * ax + ay * ay +
    //                  a_wheel[0] * a_wheel[0] + a_wheel[1] * a_wheel[1]);

    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    double ax_true = ax + w_yaw * vy; // actual rate of change of vx
    double ay_true = ay - w_yaw * vx; // actual rate of change of vy

    double score = ax_true * ax_true + ay_true * ay_true + a_yaw * a_yaw;

    std::cout << score << "\t"
              << ax << "\t"
              << u[0] << "\n";
    //   << u[1] << "\n";
    return score;
}

void stabilizeWheelSpeeds(Vehicle2D &vehicle)
{
    // stabilize wheel speeds (to solve equilibrium for non-driven wheels)
    Vehicle2DData &data = vehicle.getVehicle2DData();

    double max_a_wheel = 10.0;
    while (max_a_wheel > 0.01)
    {
        // override wheel torques to zero
        double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
        data.setWheelTorques(wheel_torques);
        // vehicle.calcTractionTorques();
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
}

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../../configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);
    Vehicle2DData &data = vehicle.getVehicle2DData();

    // optimize throttle to maintain speed
    double vx_target = 2.0;                               // [m/s]
    double vy_target = 0.0;                               // [m/s]
    // double r_turn = 0.5;                                  // [m]
    // double w_yaw_target = std::sqrt(vx_target * vx_target + vy_target * vy_target) / r_turn; // [rad/s]
    double w_yaw_target = 0.0;                            // [rad/s]
    // ax_true = 0 = ax + w_yaw * vy; // actual rate of change of vx
    // ay_true = 0 = ay - w_yaw * vx; // actual rate of change of vy
    // ax = -w_yaw * vy;
    // ay = w_yaw * vx;
    double ax = -w_yaw_target * vy_target; // [m/s^2]
    double ay = w_yaw_target * vx_target;  // [m/s^2]
    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(0.0); // [rad/s^2]
    data.setLinearVelocities(vx_target, vy_target);
    data.setAngularVelocities(w_yaw_target);

    // stabilize wheel speeds (useful to solve equilibrium for non-driven wheels)
    stabilizeWheelSpeeds(vehicle);

    // get wheel speeds when undriven
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);
    std::cout << "stable wheel speeds:\n\t"
              << w_wheel_eq[0] << "\t"
              << w_wheel_eq[1] << "\n\t"
              << w_wheel_eq[2] << "\t"
              << w_wheel_eq[3] << std::endl;

    // write data that wont change
    data.setLinearVelocities(vx_target, 0.0);
    data.setAngularVelocities(0.0);
    const double a_wheel_zero[4] = {0.0};
    data.setWheelAccelerations(a_wheel_zero);
    data.setLinearAccelerations(0.0, 0.0);
    data.setAngularAccelerations(0.0);

    // optimization
    auto f = [&vehicle](std::vector<double> x) -> double
    {
        return cost_function(vehicle, x);
    };

    OptSolver solver(f);
    solver.setStepSize(0.01);
    // solver.setThreshold(1e-5);
    solver.printOutput(false);
    solver.setMaxIters(100);
    std::vector<double> x0 = {(w_wheel_eq[2] + w_wheel_eq[3]) / 2.0};

    solver.setGuess(x0);
    if (!solver.solve())
    {
        std::cerr << "Solver did not converge." << std::endl;
        return 1;
    }

    std::vector<double> x_opt = solver.getSolution();
    std::cout << "Optimal solution: ";
    for (size_t i = 0; i < x_opt.size() - 1; ++i)
    {
        std::cout << x_opt[i] << ",";
    }
    std::cout << x_opt[x_opt.size() - 1] << std::endl;
    std::cout << "Objective function value: "
              << solver.getScore() << std::endl;

    // get traction torques

    // calculate motor torque

    // calculate throttle input command
}
