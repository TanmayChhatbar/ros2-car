#include "VehicleControlCruiseControl.hpp"

VehicleCruiseControl::VehicleCruiseControl()
{
    target_speed = 0.0;
    kp = 1.0;
    ki = 1.0;
    kd = 0.0;
    throttle_input = 0.0;
    brake_input = 0.0;
    prev_error = 0.0;
    integral = 0.0;
}
VehicleCruiseControl::VehicleCruiseControl(double target_speed_, double kp_, double ki_, double kd_)
{
    target_speed = target_speed_;
    kp = kp_;
    ki = ki_;
    kd = kd_;
    throttle_input = 0.0;
    brake_input = 0.0;
    prev_error = 0.0;
    integral = 0.0;
}
void VehicleCruiseControl::setTargetSpeed(double target_speed_)
{
    if (target_speed_ < 0.0)
        target_speed = 0.0;
    else
        target_speed = target_speed_;
}
void VehicleCruiseControl::setKp(double kp_)
{
    if (kp_ < 0.0)
        kp = 0.0;
    else
        kp = kp_;
}
void VehicleCruiseControl::setKi(double ki_)
{
    if (ki_ < 0.0)
        ki = 0.0;
    else
        ki = ki_;
}
void VehicleCruiseControl::setKd(double kd_)
{
    if (kd_ < 0.0)
        kd = 0.0;
    else
        kd = kd_;
}
void VehicleCruiseControl::calcCruiseControlInputs(const double vx)
{
    double error = target_speed - vx;
    integral += error;
    double derivative = error - prev_error;

    throttle_input = kp * error + ki * integral + kd * derivative;

    // Clamp throttle input to valid range [-1, 1]
    if (throttle_input > 1.0)
        throttle_input = 1.0;
    else if (throttle_input < -1.0)
        throttle_input = -1.0;

    prev_error = error;
}
void VehicleCruiseControl::calcCruiseControlInputs(const double vx, double &throttle_input_, double &brake_input_)
{
    calcCruiseControlInputs(vx);
    throttle_input_ = getThrottleInput();
    brake_input_ = getBrakeInput();
}
double VehicleCruiseControl::getThrottleInput()
{
    return throttle_input;
}
double VehicleCruiseControl::getBrakeInput()
{
    return brake_input;
}
