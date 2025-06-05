#include "Vehicle2D.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include "BruteSolver.hpp"
#include "CSVWriter.hpp"
#include <chrono>
#include "CSVReader.hpp"

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

    // calculate normal loads
    vehicle.calcTireNormalLoads();

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
    // double score = ax_calc * ax_calc + ay_calc * ay_calc + a_yaw_calc * a_yaw_calc;
#if DEBUG_OPTIMIZATION > 0
    std::cout << ax_true << "\t"
              << ay_true << "\t"
              << a_yaw_calc << "\n";
#endif
    (void)grad; // unused
    return score;
}

std::vector<std::vector<double>> combinations(std::vector<std::vector<double>> &values)
{
    // values is a vector of vectors, where each inner vector contains lower bound, step size, upper bound for that dimension
    std::vector<std::vector<double>> trial_points;
    std::vector<double> lb_current;
    std::vector<double> ub_current;
    std::vector<double> step_size;
    for (uint i = 0; i < values.size(); ++i)
    {
        lb_current.push_back(values[i][0]);
        step_size.push_back(values[i][1]);
        ub_current.push_back(values[i][2]);
    }

    // generate all combinations of trial points
    std::vector<double> trial_point_cur = lb_current;
    trial_points.push_back(trial_point_cur);
    while (true)
    {
        trial_point_cur[0] += step_size[0];
        for (size_t i = 0; i < (lb_current.size() - 1); ++i)
        {
            if (trial_point_cur[i] > ub_current[i])
            {
                trial_point_cur[i] = lb_current[i];
                trial_point_cur[i + 1] += step_size[i + 1];
            }
        }
        if (trial_point_cur.back() > ub_current.back())
        {
            break; // all combinations generated
        }
        trial_points.push_back(trial_point_cur);
    }
    return trial_points;
}

double optimize(Vehicle2D &vehicle, double vx_target, double vy_target)
{
    // get data and config from vehicle object
    Vehicle2DData &data = vehicle.getVehicle2DData();
    Vehicle2DConfig &config = vehicle.getVehicle2DConfig();

    double max_yaw_rate = 5.0; // [rad/s] maximum yaw rate
    double vx_front_target = vx_target;
    double vy_front_target = vy_target + max_yaw_rate * config.getA();
    double max_kinematic_steering_angle = std::atan2(vy_front_target, vx_front_target);
    double min_kinematic_steering_angle = std::atan2(vy_target, vx_target);

    data.setLinearVelocities(vx_target, vy_target);
    double wheel_speed_at_vx = vx_target / config.getWheelRadius();
    std::vector<double> lb = {
        wheel_speed_at_vx,                            // [rad/s] wheel speed
        min_kinematic_steering_angle - DEG2RAD(15.0), // [rad] steering angle
        -max_yaw_rate};                               // [rad/s] yaw rate
    std::vector<double> ub = {
        wheel_speed_at_vx * 6.0,
        max_kinematic_steering_angle + DEG2RAD(15.0),
        0.0};
    std::vector<uint> n_trials = {
        80, 80, 80};

    // set up solver
    auto f = [&vehicle](const std::vector<double> &x)
    {
        std::vector<double> grad; // not used
        Vehicle2D vehicle_copy = vehicle;
        return cost_function(x, grad, &vehicle_copy);
    };
    BruteSolver solver(f);
    solver.setBounds(lb, ub);
    solver.setNTrials(n_trials);
    solver.setNRefinements(50);
    solver.setCostThreshold(1.0e-5);
    solver.setReductionRatio(0.75);
    solver.printOutput(false);

    // optimize
    double score = 1.0e5;
    if (solver.solve())
    {
        score = solver.getScore();
        std::vector<double> opt_trial_point = solver.getSolution();
        double w_wheel[4];
        w_wheel[2] = opt_trial_point[0]; // rear wheel speed
        w_wheel[3] = opt_trial_point[0]; // rear wheel speed
        data.setWheelVelocities(w_wheel);
        data.setSteeringAngle(opt_trial_point[1]);
        data.setAngularVelocities(opt_trial_point[2]);
    }
    return score;
}

std::vector<std::vector<double>> getPointsToRefine(std::string filename, double score_threshold)
{
    CSVReader reader(filename);
    std::vector<std::vector<double>> trial_points;

    std::vector<std::vector<std::string>> data;
    if (reader.read(data))
    {
        for (uint i = 0; i < data.size(); ++i)
        {
            try
            {
                double score = std::stod(data[i][2]);
                if (score > score_threshold)
                {
                    std::vector trial_point = {std::stod(data[i][0]), std::stod(data[i][1])};
                    trial_points.push_back(trial_point);
                }
            }
            catch (const std::invalid_argument &)
            {
                // do nothing
            }
        }
        reader.close();
    }
    else
    {
        std::cerr << "Failed to read CSV file." << std::endl;
    }
    return trial_points;
}
