#include "Vehicle2D.hpp"
#include "Vehicle2DCSVWriter.hpp"
#include "Vehicle2DConfig.hpp"
#include "VehicleInput.hpp"
#include "Vehicle2DData.hpp"
#include "TireConfig.hpp"
#include "equilibrium_control_input.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);

    // prepare output CSV file
    std::string filename = "build/vehicle_sim_test.csv";
    std::ofstream output_file(filename);
    if (!output_file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }
    writeCSVHeader(output_file);

    // initial
    double throttle_input = 0.0; // [-1 to 1]
    double steering_input = 0.0; // [-1 to 1]
    double brake_input = 0.0;    // [0 to 1]

    // generate control map
    for (double vx = 1.0; vx < 10; vx+=0.25)
    {
        for (double vehicle_slip_angle = 0.5; vehicle_slip_angle < 1.5; vehicle_slip_angle += 0.25)
        {
            // find steady-state wheelspeeds and steering angle to maintain specific slip angle and velocity
            // constraints
            // - wheel acceleration == 0
            // - yaw acceleration == 0
            // inputs
            // - wheel speeds (x4)
            //   - assume constant wheel speeds for rear wheels
            // - steering angle

            // find throttle and steering until steady-state control input is found
            find_equilibrium_control_input(vehicle, vx, vehicle_slip_angle, &throttle_input, &steering_input);
            

        }
    }
    // output data
}
