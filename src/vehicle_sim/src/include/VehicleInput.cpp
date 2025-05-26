#include "VehicleInput.hpp"

VehicleInput::VehicleInput()
    : steering_input(0.0), throttle_input(0.0) {}

void VehicleInput::getSteeringInput(double &steering_input_) const
{
    steering_input_ = steering_input;
}
void VehicleInput::setSteeringInput(double steering_input_)
{
    if (steering_input_ < -1.0)
        steering_input = -1.0;
    else if (steering_input_ > 1.0)
        steering_input = 1.0;
    else
        steering_input = steering_input_;
}
void VehicleInput::getThrottleInput(double &throttle_input_) const
{
    throttle_input_ = throttle_input;
}
void VehicleInput::setThrottleInput(double throttle_input_)
{
    if (throttle_input_ < -1.0)
        throttle_input = -1.0;
    else if (throttle_input_ > 1.0)
        throttle_input = 1.0;
    else
        throttle_input = throttle_input_;
}
void VehicleInput::getBrakeInput(double &brake_input_) const
{
    brake_input_ = brake_input;
}
void VehicleInput::setBrakeInput(double brake_input_)
{
    if (brake_input_ < 0.0)
        brake_input = 0.0;
    else if (brake_input_ > 1.0)
        brake_input = 1.0;
    else
        brake_input = brake_input_;
}
