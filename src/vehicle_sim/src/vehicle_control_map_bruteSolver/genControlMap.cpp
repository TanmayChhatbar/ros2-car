#include "genControlMap.hpp"

void writeEverythingToCSV(const std::string &filename, const std::vector<std::vector<std::string>> &data)
{
    CSVWriter csv_writer = CSVWriter(filename);
    csv_writer.write(data);
    csv_writer.close();
}

int main()
{
    // load configurations
    Vehicle2DConfig config = Vehicle2DConfig::loadFromFile("../../configs/tt02.json");
    Vehicle2D vehicle = Vehicle2D(config);
    Vehicle2DData &data = vehicle.getVehicle2DData();

    // params for refinement
    bool refine_for_score = true;
    double score_threshold = 0.05; // [-]

    bool refine_for_steering_angle = true;
    double steering_angle_threshold = 1.0; // [rad]

    bool refine_for_wheel_speed = true;
    double wheel_speed_ratio_threshold = 5.8 / config.getWheelRadius(); // [(rad/s)/(m/s)]

    // get existing data
    std::string filename = "../../control_map_bruteSolver.csv";
    CSVReader reader(filename);
    std::vector<std::vector<std::string>> csv_data;
    if (!reader.read(csv_data))
    {
        std::cout << "CSV file not found or empty, creating new file\n";
        CSVWriter csv_writer = CSVWriter(filename);
        csv_data.push_back({"vx_target [m/s]",
                            "vy_target [m/s]",
                            "opt_score [-]",
                            "opt_wheelspeed [rad/s]",
                            "opt_steering_angle [rad]",
                            "opt_yawrate [rad/s]"});

        std::vector<std::vector<double>> values = {
            {0.25, 0.25, 10.0}, // vx
            {0.0, 0.1, 2.01},   // vy / vx
        };
        std::vector<std::vector<double>> trial_points = combinations(values);
        for (uint i = 0; i < trial_points.size(); ++i)
        {
            double vx_target = trial_points[i][0];
            double vy_target = trial_points[i][1] * vx_target;
            csv_data.push_back({std::to_string(vx_target),
                                std::to_string(vy_target / vx_target),
                                "1.0e5",   // initial score
                                "1.0e5",   // wheel speed
                                "1.0e5",   // steering angle
                                "1.0e5"}); // yaw rate
        }
        writeEverythingToCSV(filename, csv_data);
    }

    uint n_points = csv_data.size();
    for (uint i = 0; i < n_points; ++i)
    {
        double vx_target = 0.0;
        double vy_target = 0.0;
        double previous_score = 1.0e5;
        double previous_steering_angle = 0.0;
        double previous_wheel_speed = 0.0;
        double previous_yaw_rate = 0.0;
        try
        {
            vx_target = std::stod(csv_data[i][0]);
            vy_target = std::stod(csv_data[i][1]) * vx_target;
            previous_score = std::stod(csv_data[i][2]);
            previous_wheel_speed = std::stod(csv_data[i][3]);
            previous_steering_angle = std::stod(csv_data[i][4]);
            previous_yaw_rate = std::stod(csv_data[i][5]);
        }
        catch (const std::invalid_argument &)
        {
            std::cout << i + 1 << "/" << n_points << ": Invalid line (" << csv_data[i][2] << ")\n";
            continue;
        }
        // condition to skip refinement
        if (((previous_score < score_threshold) || !refine_for_score) &&
            ((std::fabs(previous_steering_angle) < steering_angle_threshold) || !refine_for_steering_angle) &&
            ((std::fabs(previous_wheel_speed) < wheel_speed_ratio_threshold * vx_target) || !refine_for_wheel_speed))
        {
            std::cout << i + 1 << "/" << n_points << ": No need to optimize (" << previous_score << ")\n";
            continue;
        }
        std::cout << i + 1 << "/" << n_points << ": " << std::flush;

        double opt_wheel_speed;
        double opt_steering_angle = data.getSteeringAngle();
        double opt_yaw_rate;
        double opt_score = optimize(vehicle, vx_target, vy_target);
        double opt_wheel_speeds_tmp[4];
        data.getWheelVelocities(opt_wheel_speeds_tmp);
        opt_wheel_speed = opt_wheel_speeds_tmp[2];
        data.getAngularVelocities(opt_yaw_rate);

        (void)previous_yaw_rate;
        if (previous_score > opt_score ||
            fabsf(previous_steering_angle) > steering_angle_threshold ||
            fabsf(previous_wheel_speed) > wheel_speed_ratio_threshold * vx_target)
        {
            csv_data[i][0] = std::to_string(vx_target);
            csv_data[i][1] = std::to_string(vy_target / vx_target);
            csv_data[i][2] = std::to_string(opt_score);
            csv_data[i][3] = std::to_string(opt_wheel_speed);
            csv_data[i][4] = std::to_string(opt_steering_angle);
            csv_data[i][5] = std::to_string(opt_yaw_rate);
            std::cout << "Updated score (" << opt_score << " --> " << previous_score << ")\n";
            writeEverythingToCSV(filename, csv_data);
        }
        else
        {
            std::cout << "No update (" << opt_score << " > " << previous_score << ")\n";
        }
    }
    return 0;

    // print wheel speeds

    // get traction torques

    // calculate motor torque

    // calculate throttle input command
}
