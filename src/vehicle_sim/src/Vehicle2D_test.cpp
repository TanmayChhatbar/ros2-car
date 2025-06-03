#include "Vehicle2D.hpp"
#include "Vehicle2DCSVWriter.hpp"
#include "Vehicle2DConfig.hpp"
#include "VehicleInput.hpp"
#include "Vehicle2DData.hpp"
#include "TireConfig.hpp"
#include "VehicleControlCruiseControl.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("configs/toyota_86.json");
    Vehicle2D vehicle = Vehicle2D(config);
    VehicleCruiseControl cruise_control = VehicleCruiseControl(10, 1, 0.0, 0.5);

    // prepare output CSV file
    std::string filename = "build/vehicle_sim_test.csv";
    std::ofstream output_file(filename);
    if (!output_file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }
    writeCSVHeader(output_file);

    // simulation parameters
    double max_steering_input = 0.1;
    double dt = 0.0001;      // [s]
    double tmax = 2000.0;      // [s]
    double freq_steer = 0.1; // frequency of steering oscillation
    double t_start = 1;
    double f_log = 100;      // [hz] log frequency

    // initial
    double current_time = 0.0;
    int step_count = 0;
    double throttle_input = 1.0; // [-1 to 1]
    double steering_input = 0.0; // [-1 to 1]
    double brake_input = 0.0;    // [0 to 1]
    vehicle.getVehicle2DData().setLinearVelocities(1.0, 0.0);

    // run simulation
    std::cout << "Running test..." << std::endl;
    for (int step = 0; step < (int)(tmax / dt); step++)
    {
        // calculate inputs based on current times
        if (current_time > t_start && current_time < t_start + 2 / freq_steer)
        {
            steering_input = max_steering_input * std::sin(freq_steer * (current_time - t_start) * 2 * M_PI);
        }
        else
        {
            steering_input = 0.0;
        }
        // cruise control
        double w_wheel[4];
        vehicle.getVehicle2DData().getWheelVelocities(w_wheel);
        double vx_est = (w_wheel[2] + w_wheel[3]) / 2.0 * vehicle.getVehicle2DConfig().getWheelRadius();
        cruise_control.calcCruiseControlInputs(vx_est, throttle_input, brake_input);

        // brake test
        // if (current_time > t_start + 3 && current_time < t_start + 3.5)
        //     brake_input = 1.0;
        // else
        //     brake_input = 0.0;

        // set inputs and step simulation
        vehicle.stepSimulation(dt, steering_input, throttle_input, brake_input);

        // output data
        if (step_count % (int)(1 / (f_log * dt)) == 0)
        {
            writeCSVData(output_file, current_time, vehicle.getVehicle2DData(), vehicle.getVehicleInput());
        }

        current_time += dt;
        step_count++;
    }

    output_file.close();
    std::cout << "Finished test. Data saved to " << filename << std::endl;

    return 0;
}
