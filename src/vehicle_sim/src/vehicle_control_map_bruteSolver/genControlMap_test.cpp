#include "genControlMap_test.hpp"

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../../configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);
    Vehicle2DData &data = vehicle.getVehicle2DData();

    // optimize throttle to maintain speed
    std::vector<std::vector<double>> values = {
        {0.25, 0.25, 10.0}, // vx
        {0.0, 0.1, 2.01},   // vy / vx
    };
    std::vector<std::vector<double>> trial_points = combinations(values);
    uint n_points = trial_points.size();

    CSVWriter csv_writer = CSVWriter("control_map.csv");
    csv_writer.write({"vx_target [m/s]",
                      "vy_target [m/s]",
                      "opt_score [-]",
                      "opt_wheelspeed [rad/s]",
                      "opt_steering_angle [rad]",
                      "opt_yawrate [rad/s]"});

    uint i_point = 0;
    for (const auto &trial_point : trial_points)
    {
        i_point++;
        std::cout << i_point << "/" << n_points << ": " << std::flush;
        double vx_target = trial_point[0];
        double vy_target = trial_point[1] * vx_target;

        double opt_wheel_speed;
        double opt_steering_angle = data.getSteeringAngle();
        double opt_yaw_rate;
        // auto t1 = std::chrono::high_resolution_clock::now();
        double opt_score = optimize(vehicle, vx_target, vy_target);
        // auto t2 = std::chrono::high_resolution_clock::now();
        // double time = std::chrono::duration<double>(t2 - t1).count();
        double opt_wheel_speeds_tmp[4];
        data.getWheelVelocities(opt_wheel_speeds_tmp);
        opt_wheel_speed = opt_wheel_speeds_tmp[2];
        data.getAngularVelocities(opt_yaw_rate);
        std::cout << opt_score << "\n";
        // std::cout << "Optimization time: " << time << " s\n";
        std::vector<double> to_print = {
            vx_target, vy_target, opt_score, opt_wheel_speed, opt_steering_angle, opt_yaw_rate};
        csv_writer.write(to_print);
    }
    csv_writer.close();
    return 0;

    // print wheel speeds

    // get traction torques

    // calculate motor torque

    // calculate throttle input command
}
