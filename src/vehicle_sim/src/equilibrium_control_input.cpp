#include "equilibrium_control_input.hpp"

void find_equilibrium_control_input(Vehicle2D vehicle, const double vx, const double vehicle_slip_angle, double *throttle_input, double *steering_input)
{
    const double vy = vx * std::tan(vehicle_slip_angle);

    // get simulation data
    VehicleInput input = vehicle.getVehicleInput();
    Vehicle2DConfig config = vehicle.getVehicle2DConfig();
    Vehicle2DData data = vehicle.getVehicle2DData();

    // set initial conditions
    data.setLinearVelocities(vx, vy);
    data.setAngularVelocities(0.0);

    // set control inputs
    double steering_input_trial = 0.0;
    double throttle_input_trial = 0.0;
    input.setSteeringInput(steering_input_trial);
    input.setThrottleInput(throttle_input_trial);
    input.setBrakeInput(0.0);

    // solve most of the simulation
    vehicle.calcSteeringAngle();
    vehicle.calcMotorTorque();
    vehicle.calcBrakeTorque();
    if (config.getDrivetrainType() == RWD)
    {
        vehicle.calcTractionTorquesRWD();
    }
    else if (config.getDrivetrainType() == FWD)
    {
        vehicle.calcTractionTorquesFWD();
    }
    else if (config.getDrivetrainType() == AWD)
    {
        vehicle.calcTractionTorquesAWD();
    }
    vehicle.calcTireNormalLoads();
    vehicle.calcWheelSlipsAndForces();
    vehicle.calcNetForcesAndMoments();
    vehicle.calcBodyAccelerations();
    vehicle.calcWheelAccelerations();

    double a_wheel[4];
    double ax, ay, a_yaw;
    data.getWheelAccelerations(a_wheel);
    data.getLinearAccelerations(ax, ay);
    data.getAngularAccelerations(a_yaw);

    // condition for equilibrium
    

}
