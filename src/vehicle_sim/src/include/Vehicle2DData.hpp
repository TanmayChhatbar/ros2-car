#pragma once

class Vehicle2DData
{
public:
    Vehicle2DData();

    double getSteeringAngle() const;
    void setSteeringAngle(const double steering_angle_);
    double getMotorTorque() const;
    void setMotorTorque(const double motor_torque_);
    void getBrakeTorque(double (&brake_torque_)[4]) const;
    void setBrakeTorque(const double brake_torque_[4]);

    void getPosition(double &x_, double &y_) const;
    void setPosition(const double x_, const double y_);
    void getOrientation(double &yaw_) const;
    void setOrientation(const double yaw_);

    void getLinearVelocities(double &vx_, double &vy_) const;
    void setLinearVelocities(const double vx_, const double vy_);
    void getAngularVelocities(double &w_yaw_) const;
    void setAngularVelocities(const double w_yaw_);

    void getWheelVelocities(double &w_wheel_fl_, double &w_wheel_fr_, double &w_wheel_rl_, double &w_wheel_rr_) const;
    void setWheelVelocities(const double w_wheel_fl_, const double w_wheel_fr_, const double w_wheel_rl_, const double w_wheel_rr_);
    void getWheelVelocities(double (&w_wheel_)[4]) const;
    void setWheelVelocities(const double w_wheel_[4]);
    void getWheelRotations(double (&p_wheel_)[4]) const;
    void setWheelRotations(const double p_wheel_[4]);

    void getLinearAccelerations(double &ax_, double &ay_) const;
    void setLinearAccelerations(const double ax_, const double ay_);
    void getAngularAccelerations(double &a_yaw_) const;
    void setAngularAccelerations(const double a_yaw_);
    void getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_) const;
    void setWheelAccelerations(const double a_fl_, const double a_fr_, const double a_rl_, const double a_rr_);
    void getWheelAccelerations(double (&a_wheel_)[4]) const;
    void setWheelAccelerations(const double a_wheel_[4]);

    void getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const;
    void setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_);
    void getWheelNormalLoads(double (&Fz_wheel_)[4]) const;
    void setWheelNormalLoads(const double Fz_wheel_[4]);
    void getWheelForces(double (&Fx_wheel_)[4], double (&Fy_wheel_)[4]) const;
    void setWheelForces(const double Fx_wheel_[4], const double Fy_wheel_[4]);
    void getWheelTorques(double (&wheel_torques_)[4]) const;
    void setWheelTorques(const double (&wheel_torques_)[4]);

private:
    double X, Y, yaw;     // global 2D position and orientation
    double vx, vy, w_yaw; // vehicle frame velocities
    double ax, ay, a_yaw; // vehicle frame accelerations

    double p_wheel[4];
    double w_wheel[4];
    double a_wheel[4];

    double Fx, Fy, Mz;
    double Fx_wheel[4];
    double Fy_wheel[4];
    double Fz_wheel[4];

    double steering_angle;
    double motor_torque;
    double brake_torque[4];
    double wheel_torques[4];

    friend class Vehicle2D;
};
