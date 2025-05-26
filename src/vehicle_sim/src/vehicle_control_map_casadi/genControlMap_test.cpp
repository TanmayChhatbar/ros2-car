#include "Vehicle2D.hpp"
#include "Vehicle2DConfig.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <casadi/casadi.hpp>

casadi::MX cost_function(Vehicle2D& vehicle, const casadi::MX& u) {
    // cost function
    // input - rear wheel speed (assumed same)
    // output - score based on accelerations

    Vehicle2DData data = vehicle.getVehicle2DData();
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);

    double w_wheel_in = u; // input for rear wheel speeds
    // double w_wheel_in = (double) u; // input for rear wheel speeds
    double w_wheel[4] = {w_wheel_eq[0], w_wheel_eq[1], w_wheel_in, w_wheel_in}; // only rear wheels are driven

    // calculate accelerations
    data.setWheelVelocities(w_wheel);
    vehicle.calcTireNormalLoads();
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcNetForcesAndMoments();
    vehicle.calcBodyAccelerations();
    vehicle.calcWheelAccelerations();

    // get accelerations
    double ax, ay;
    double a_wheel[4];
    data.getLinearAccelerations(ax, ay);
    data.getWheelAccelerations(a_wheel);

    // calculate score
    double score = ax * ax + ay * ay +
                    a_wheel[0] * a_wheel[0] + a_wheel[1] * a_wheel[1] +
                    a_wheel[2] * a_wheel[2] + a_wheel[3] * a_wheel[3];
    
    return casadi::MX(score);
}

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../../configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);
    Vehicle2DData data = vehicle.getVehicle2DData();

    // optimize throttle to maintain speed
    double target_speed = 5.0; // target speed in m/s

    // stabilize wheel speeds (useful to solve equilibrium for non-driven wheels)
    for (uint i = 0; i < 1000; ++i)
    {
        // calculate equilibrium control input
        data.setLinearVelocities(target_speed, 0.0);
        data.setAngularVelocities(0.0);
        vehicle.stepSimulation(0.001, 0.0, 0.0, 0.0);
    }
    double w_wheel_eq[4];
    data.getWheelVelocities(w_wheel_eq);

    // write data that wont change
    data.setLinearVelocities(target_speed, 0.0);
    data.setAngularVelocities(0.0);
    const double a_wheel_zero[4] = {0.0};
    data.setWheelAccelerations(a_wheel_zero);
    data.setLinearAccelerations(0.0, 0.0);
    data.setAngularAccelerations(0.0);

    // optimization
    casadi::MX u = casadi::MX::sym("x"); // control input (rear wheel speed)
    casadi::MX f = cost_function(vehicle, u);
    casadi::MXDict nlp = {{"x", u}, {"f", f}};

    casadi::Function solver = casadi::nlpsol("solver", "ipopt", nlp);
    
    // Set bounds and initial guess
    casadi::DMDict arg = {  {"lbx", -1000},
                            {"ubx", 1000},
                            {"x0", 5.0},
                        };
    casadi::DMDict res = solver(arg);

    // Output result
    double optimal_w = static_cast<double>(res.at("x"));
    std::cout << "Optimal rear wheel speed: " << optimal_w << std::endl;
}
