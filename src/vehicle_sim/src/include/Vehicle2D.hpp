#pragma once

#include "TireConfig.hpp"
#include "Vehicle2DData.hpp"
#include "Vehicle2DConfig.hpp"
#include "VehicleInput.hpp"
#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

class Vehicle2D
{
public:
    Vehicle2D();
    Vehicle2D(const Vehicle2DConfig &config_);
    Vehicle2D(const Vehicle2DData &data_, const Vehicle2DConfig &config_, const VehicleInput &input_);

    void calcMotorTorque();
    void calcBrakeTorque();
    void calcSteeringAngle();
    void calcTractionTorques();
    void calcTractionTorquesFWD();
    void calcTractionTorquesRWD();
    void calcTractionTorquesAWD();
    void calcWheelSlipsAndForces();
    void calcTireNormalLoads();
    void calcAerodynamicForces(double (&F)[3], double (&M)[3]);
    void calcNetForcesAndMoments();
    void calcBodyAccelerations();
    void calcWheelAccelerations();
    void calcNewState(double dt);
    bool stepSimulation(double dt, double steering_input, double throttle_input, double brake_input);
    Vehicle2DData &getVehicle2DData();
    Vehicle2DConfig &getVehicle2DConfig();
    VehicleInput &getVehicleInput();

private:
    Vehicle2DData data;
    Vehicle2DConfig config;
    VehicleInput input;
};
