#pragma once

#include "Vehicle2D.hpp"

class VehicleCruiseControl
{
public:
    VehicleCruiseControl();
    VehicleCruiseControl(double target_speed_, double kp_, double ki_, double kd_);
    void setTargetSpeed(double target_speed_);
    void setKp(double kp_);
    void setKi(double ki_);
    void setKd(double kd_);
    void calcCruiseControlInputs(const double vx);
    void calcCruiseControlInputs(const double vx, double &throttle_input_, double &brake_input_);
    double getThrottleInput();
    double getBrakeInput();

private:
    double target_speed;   // [m/s] desired speed
    double kp;             // proportional gain
    double ki;             // integral gain
    double kd;             // derivative gain
    double throttle_input; // [-1, 1] throttle input
    double brake_input;    // [-1, 1] steering input
    double prev_error;     // previous error for derivative calculation
    double integral;       // integral of error for integral calculation
};
