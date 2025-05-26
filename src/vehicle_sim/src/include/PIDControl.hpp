class PIDController
{
public:
    PIDController(double kp_, double ki_, double kd_, double dt_, double windup_limit_);

    double compute(double setpoint, double measured_value);
    void reset();
    void setWindupLimit(double limit);

private:
    double kp;           // proportional gain
    double ki;           // integral gain
    double kd;           // derivative gain
    double dt;           // time step
    double prev_error;   // previous error
    double integral;     // integral of error
    double windup_limit; // anti-windup limit
};
