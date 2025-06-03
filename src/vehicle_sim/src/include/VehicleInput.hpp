#pragma once

class VehicleInput // from user/controller
{
public:
    VehicleInput();

    void getSteeringInput(double &steering_input_) const;
    void setSteeringInput(double steering_input_);
    void getThrottleInput(double &throttle_input_) const;
    void setThrottleInput(double throttle_input_);
    void getBrakeInput(double &brake_input_) const;
    void setBrakeInput(double brake_input_);

private:
    double steering_input; // [rad] steering angle
    double throttle_input; // [-1, 1] throttle input
    double brake_input;    // [0, 1] brake input

    friend class Vehicle2D; // allow Vehicle to access private members
};
