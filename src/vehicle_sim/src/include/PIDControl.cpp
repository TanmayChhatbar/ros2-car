#include "PIDControl.hpp"

PIDController::PIDController(double kp_, double ki_, double kd_, double dt_, double windup_limit_)
    : kp(kp_), ki(ki_), kd(kd_), dt(dt_), prev_error(0.0), integral(0.0), windup_limit(windup_limit_) {}

double PIDController::compute(double setpoint, double measured_value)
{
    double error = setpoint - measured_value;

    integral += error * dt;

    // apply anti-windup by clamping the integral term
    if (windup_limit > 0.0)
    {
        if (integral > windup_limit)
        {
            integral = windup_limit;
        }
        else if (integral < -windup_limit)
        {
            integral = -windup_limit;
        }
    }

    double derivative = (error - prev_error) / dt;
    prev_error = error;

    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset()
{
    prev_error = 0.0;
    integral = 0.0;
}

void PIDController::setWindupLimit(double limit)
{
    windup_limit = limit;
}
