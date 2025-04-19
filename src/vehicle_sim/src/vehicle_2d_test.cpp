#include "vehicle_2d.cpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

// Basic driving scenario definitions
struct DrivingScenario {
    double duration;   // total simulation time in seconds
    double dt;         // time step in seconds
    std::function<double(double)> steering_input;  // steering input function (time -> [-1,1])
    std::function<double(double)> throttle_input;  // throttle input function (time -> [-1,1])
};

// CSV header and data formatting
void writeCSVHeader(std::ofstream& file) {
    file << "time,X,Y,yaw,vx,vy,w_yaw,ax,ay,a_yaw,"
         << "v_fl,v_fr,v_rl,v_rr,"
         << "Fx_wheel_fl,Fx_wheel_fr,Fx_wheel_rl,Fx_wheel_rr,"
         << "Fy_wheel_fl,Fy_wheel_fr,Fy_wheel_rl,Fy_wheel_rr,"
         << "steering,throttle" << std::endl;
}

void writeCSVData(std::ofstream& file, double time, const VehicleData& data, 
                  double steering_input, double throttle_input) {
    double X, Y, yaw;
    double vx, vy, w_yaw;
    double ax, ay, a_yaw;
    double v_wheel[4];
    double Fx_wheel[4], Fy_wheel[4];

    data.getPosition(X, Y);
    data.getOrientation(yaw);
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    data.getLinearAccelerations(ax, ay);
    data.getAngularAccelerations(a_yaw);
    data.getWheelVelocities(v_wheel);
    data.getWheelForces(Fx_wheel, Fy_wheel);

    file << time << "," 
         << X << "," << Y << "," << yaw << ","
         << vx << "," << vy << "," << w_yaw << ","
         << ax << "," << ay << "," << a_yaw << ","
         << v_wheel[0] << "," << v_wheel[1] << "," << v_wheel[2] << "," << v_wheel[3] << ","
         << Fx_wheel[0] << "," << Fx_wheel[1] << "," << Fx_wheel[2] << "," << Fx_wheel[3] << ","
         << Fy_wheel[0] << "," << Fy_wheel[1] << "," << Fy_wheel[2] << "," << Fy_wheel[3] << ","
         << steering_input << "," << throttle_input << std::endl;
}

// Configure a realistic vehicle
VehicleConfig createDefaultVehicleConfig() {
    // Example vehicle parameters - these represent a typical passenger car
    double wheelbase = 2.7;       // m
    double track_width = 1.6;     // m
    double steer_max = 0.5;       // rad (~28.6 degrees)
    double mass = 1500.0;         // kg
    double Izz = 2500.0;          // kg·m²
    double z_cg = 0.5;            // m
    double a = 1.35;              // m (distance from front axle to CG)
    double r_wheel = 0.3;         // m
    double I_wheel = 1.0;         // kg·m²
    double Tmax = 1500.0;         // Nm
    double diff_damping = 10.0;   // Nm·s/rad

    // Pacejka tire model parameters
    TireConfig tire_config(10.0, 1.9, 1.0, 0.97, 0.7);

    return VehicleConfig(
        wheelbase, track_width, steer_max,
        mass, Izz, z_cg, a, r_wheel, I_wheel,
        Tmax, diff_damping, tire_config
    );
}

// Define some driving scenarios
std::vector<DrivingScenario> createDrivingScenarios() {
    std::vector<DrivingScenario> scenarios;
    
    // Scenario 1: Straight line acceleration
    scenarios.push_back({
        10.0,  // 10 seconds
        0.01,  // 10ms time step
        [](double t) { return 0.0; },  // no steering
        [](double t) { return t < 1.0 ? t : 1.0; }  // ramp up throttle to full
    });
    
    // Scenario 2: Lane change
    scenarios.push_back({
        10.0,  // 10 seconds
        0.01,  // 10ms time step
        [](double t) {
            // Sinusoidal steering for lane change
            if (t >= 2.0 && t < 6.0) {
                return 0.5 * std::sin((t - 2.0) * M_PI / 2.0);
            }
            return 0.0;
        },
        [](double t) { return 0.5; }  // constant half throttle
    });
    
    // Scenario 3: Figure-8 maneuver
    scenarios.push_back({
        20.0,  // 20 seconds
        0.01,  // 10ms time step
        [](double t) {
            // Sinusoidal steering for figure-8
            return 0.7 * std::sin(t * M_PI / 5.0);
        },
        [](double t) { return 0.3; }  // constant low throttle
    });
    
    return scenarios;
}

int main() {
    // Create vehicle with default configuration
    VehicleConfig config = createDefaultVehicleConfig();
    Vehicle vehicle(config);
    
    // Create and run multiple driving scenarios
    std::vector<DrivingScenario> scenarios = createDrivingScenarios();
    
    for (size_t scenario_idx = 0; scenario_idx < scenarios.size(); scenario_idx++) {
        const auto& scenario = scenarios[scenario_idx];
        
        // Reset vehicle for each scenario
        vehicle = Vehicle(config);
        
        // Prepare output CSV file
        std::string filename = "vehicle_sim_scenario_" + std::to_string(scenario_idx+1) + ".csv";
        std::ofstream output_file(filename);
        
        if (!output_file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            continue;
        }
        
        writeCSVHeader(output_file);
        
        // Run simulation
        double current_time = 0.0;
        int step_count = 0;
        
        std::cout << "Running scenario " << scenario_idx+1 << "..." << std::endl;
        
        while (current_time < scenario.duration) {
            // Get inputs based on current time
            double steering_input = scenario.steering_input(current_time);
            double throttle_input = scenario.throttle_input(current_time);
            
            // Set inputs and step simulation
            vehicle.setSteeringInput(steering_input);
            vehicle.setThrottleInput(throttle_input);
            vehicle.stepSimulation(scenario.dt);
            
            // Output data every 10 steps (reduces file size)
            if (step_count % 10 == 0) {
                writeCSVData(output_file, current_time, vehicle.getData(), steering_input, throttle_input);
            }
            
            current_time += scenario.dt;
            step_count++;
        }
        
        output_file.close();
        std::cout << "Finished scenario " << scenario_idx+1 << ", data saved to " << filename << std::endl;
    }
    
    std::cout << "All simulations complete!" << std::endl;
    return 0;
}
