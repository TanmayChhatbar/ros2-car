#include "Vehicle2D.hpp"

Vehicle2DData::Vehicle2DData()
    : X(0.0), Y(0.0), yaw(0.0),
      vx(0.0), vy(0.0), w_yaw(0.0),
      ax(0.0), ay(0.0), a_yaw(0.0),
      p_wheel{0.0, 0.0, 0.0, 0.0},
      w_wheel{0.0, 0.0, 0.0, 0.0},
      a_wheel{0.0, 0.0, 0.0, 0.0},
      Fx(0.0), Fy(0.0), Mz(0.0),
      Fx_wheel{0.0, 0.0, 0.0, 0.0},
      Fy_wheel{0.0, 0.0, 0.0, 0.0},
      Fz_wheel{0.0, 0.0, 0.0, 0.0},
      steering_angle(0.0), motor_torque(0.0),
      wheel_torques{0.0, 0.0, 0.0, 0.0} {}

// getters and setters for inputs
double Vehicle2DData::getSteeringAngle() const { return steering_angle; }
void Vehicle2DData::setSteeringAngle(const double steering_angle_) { steering_angle = steering_angle_; }
double Vehicle2DData::getMotorTorque() const { return motor_torque; }
void Vehicle2DData::setMotorTorque(const double motor_torque_) { motor_torque = motor_torque_; }
void Vehicle2DData::getBrakeTorque(double (&brake_torque_)[4]) const
{
    brake_torque_[0] = brake_torque[0];
    brake_torque_[1] = brake_torque[1];
    brake_torque_[2] = brake_torque[2];
    brake_torque_[3] = brake_torque[3];
}
void Vehicle2DData::setBrakeTorque(const double brake_torque_[4])
{
    brake_torque[0] = brake_torque_[0];
    brake_torque[1] = brake_torque_[1];
    brake_torque[2] = brake_torque_[2];
    brake_torque[3] = brake_torque_[3];
}

// getters and setters for position and orientation
void Vehicle2DData::getPosition(double &x_, double &y_) const
{
    x_ = X;
    y_ = Y;
}
void Vehicle2DData::setPosition(const double x_, const double y_)
{
    X = x_;
    Y = y_;
}
void Vehicle2DData::getOrientation(double &yaw_) const { yaw_ = yaw; }
void Vehicle2DData::setOrientation(const double yaw_) { yaw = yaw_; }

// getters and setters for velocities
void Vehicle2DData::getLinearVelocities(double &vx_, double &vy_) const
{
    vx_ = vx;
    vy_ = vy;
}
void Vehicle2DData::setLinearVelocities(const double vx_, const double vy_)
{
    vx = vx_;
    vy = vy_;
}
void Vehicle2DData::getAngularVelocities(double &w_yaw_) const { w_yaw_ = w_yaw; }
void Vehicle2DData::setAngularVelocities(const double w_yaw_) { w_yaw = w_yaw_; }

void Vehicle2DData::getWheelVelocities(double &w_wheel_fl_, double &w_wheel_fr_, double &w_wheel_rl_, double &w_wheel_rr_) const
{
    w_wheel_fl_ = w_wheel[0];
    w_wheel_fr_ = w_wheel[1];
    w_wheel_rl_ = w_wheel[2];
    w_wheel_rr_ = w_wheel[3];
}
void Vehicle2DData::setWheelVelocities(const double w_wheel_fl_, const double w_wheel_fr_, const double w_wheel_rl_, const double w_wheel_rr_)
{
    w_wheel[0] = w_wheel_fl_;
    w_wheel[1] = w_wheel_fr_;
    w_wheel[2] = w_wheel_rl_;
    w_wheel[3] = w_wheel_rr_;
}
void Vehicle2DData::getWheelVelocities(double (&w_wheel_)[4]) const
{
    w_wheel_[0] = w_wheel[0];
    w_wheel_[1] = w_wheel[1];
    w_wheel_[2] = w_wheel[2];
    w_wheel_[3] = w_wheel[3];
}
void Vehicle2DData::setWheelVelocities(const double w_wheel_[4])
{
    w_wheel[0] = w_wheel_[0];
    w_wheel[1] = w_wheel_[1];
    w_wheel[2] = w_wheel_[2];
    w_wheel[3] = w_wheel_[3];
}
void Vehicle2DData::getWheelRotations(double (&p_wheel_)[4]) const
{
    p_wheel_[0] = p_wheel[0];
    p_wheel_[1] = p_wheel[1];
    p_wheel_[2] = p_wheel[2];
    p_wheel_[3] = p_wheel[3];
}
void Vehicle2DData::setWheelRotations(const double p_wheel_[4])
{
    p_wheel[0] = p_wheel_[0];
    p_wheel[1] = p_wheel_[1];
    p_wheel[2] = p_wheel_[2];
    p_wheel[3] = p_wheel_[3];
}

// getters and setters for accelerations
void Vehicle2DData::getLinearAccelerations(double &ax_, double &ay_) const
{
    ax_ = ax;
    ay_ = ay;
}
void Vehicle2DData::setLinearAccelerations(const double ax_, const double ay_)
{
    ax = ax_;
    ay = ay_;
}
void Vehicle2DData::getAngularAccelerations(double &a_yaw_) const { a_yaw_ = a_yaw; }
void Vehicle2DData::setAngularAccelerations(const double a_yaw_) { a_yaw = a_yaw_; }
void Vehicle2DData::getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_) const
{
    a_fl_ = a_wheel[0];
    a_fr_ = a_wheel[1];
    a_rl_ = a_wheel[2];
    a_rr_ = a_wheel[3];
}
void Vehicle2DData::setWheelAccelerations(const double a_fl_, const double a_fr_, const double a_rl_, const double a_rr_)
{
    a_wheel[0] = a_fl_;
    a_wheel[1] = a_fr_;
    a_wheel[2] = a_rl_;
    a_wheel[3] = a_rr_;
}
void Vehicle2DData::getWheelAccelerations(double (&a_wheel_)[4]) const
{
    a_wheel_[0] = a_wheel[0];
    a_wheel_[1] = a_wheel[1];
    a_wheel_[2] = a_wheel[2];
    a_wheel_[3] = a_wheel[3];
}
void Vehicle2DData::setWheelAccelerations(const double a_wheel_[4])
{
    a_wheel[0] = a_wheel_[0];
    a_wheel[1] = a_wheel_[1];
    a_wheel[2] = a_wheel_[2];
    a_wheel[3] = a_wheel_[3];
}

// getters and setters for forces and moments
void Vehicle2DData::getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const
{
    Fx_ = Fx;
    Fy_ = Fy;
    Mz_ = Mz;
}
void Vehicle2DData::setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_)
{
    Fx = Fx_;
    Fy = Fy_;
    Mz = Mz_;
}
void Vehicle2DData::getWheelNormalLoads(double (&Fz_wheel_)[4]) const
{
    Fz_wheel_[0] = Fz_wheel[0];
    Fz_wheel_[1] = Fz_wheel[1];
    Fz_wheel_[2] = Fz_wheel[2];
    Fz_wheel_[3] = Fz_wheel[3];
}
void Vehicle2DData::setWheelNormalLoads(const double Fz_wheel_[4])
{
    Fz_wheel[0] = Fz_wheel_[0];
    Fz_wheel[1] = Fz_wheel_[1];
    Fz_wheel[2] = Fz_wheel_[2];
    Fz_wheel[3] = Fz_wheel_[3];
}
void Vehicle2DData::getWheelForces(double (&Fx_wheel_)[4], double (&Fy_wheel_)[4]) const
{
    Fx_wheel_[0] = Fx_wheel[0];
    Fx_wheel_[1] = Fx_wheel[1];
    Fx_wheel_[2] = Fx_wheel[2];
    Fx_wheel_[3] = Fx_wheel[3];
    Fy_wheel_[0] = Fy_wheel[0];
    Fy_wheel_[1] = Fy_wheel[1];
    Fy_wheel_[2] = Fy_wheel[2];
    Fy_wheel_[3] = Fy_wheel[3];
}
void Vehicle2DData::setWheelForces(const double Fx_wheel_[4], const double Fy_wheel_[4])
{
    Fx_wheel[0] = Fx_wheel_[0];
    Fx_wheel[1] = Fx_wheel_[1];
    Fx_wheel[2] = Fx_wheel_[2];
    Fx_wheel[3] = Fx_wheel_[3];
    Fy_wheel[0] = Fy_wheel_[0];
    Fy_wheel[1] = Fy_wheel_[1];
    Fy_wheel[2] = Fy_wheel_[2];
    Fy_wheel[3] = Fy_wheel_[3];
}
void Vehicle2DData::getWheelTorques(double (&wheel_torques_)[4]) const
{
    wheel_torques_[0] = wheel_torques[0];
    wheel_torques_[1] = wheel_torques[1];
    wheel_torques_[2] = wheel_torques[2];
    wheel_torques_[3] = wheel_torques[3];
}
void Vehicle2DData::setWheelTorques(const double (&wheel_torques_)[4])
{
    wheel_torques[0] = wheel_torques_[0];
    wheel_torques[1] = wheel_torques_[1];
    wheel_torques[2] = wheel_torques_[2];
    wheel_torques[3] = wheel_torques_[3];
}
