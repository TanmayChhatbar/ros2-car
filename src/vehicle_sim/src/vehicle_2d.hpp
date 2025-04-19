#pragma once

#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

class VehicleInput // from user/controller
{
public:
    VehicleInput();

    void getSteeringInput(double &steering_input_) const;
    void setSteeringInput(double steering_input_);
    void getThrottleInput(double &throttle_input_) const;
    void setThrottleInput(double throttle_input_);

private:
    double steering_input; // [rad] steering angle
    double throttle_input; // [0, 1] throttle_input input

    friend class Vehicle; // allow Vehicle to access private members
};

class VehicleData
{
public:
    VehicleData();

    double getSteeringAngle() const;
    void setSteeringAngle(const double steering_angle_);
    double getMotorTorque() const;
    void setMotorTorque(const double motor_torque_);

    void getPosition(double &x_, double &y_) const;
    void setPosition(const double x_, const double y_);
    void getOrientation(double &yaw_) const;
    void setOrientation(const double yaw_);

    void getLinearVelocities(double &vx_, double &vy_) const;
    void setLinearVelocities(const double vx_, const double vy_);
    void getAngularVelocities(double &w_yaw_) const;
    void setAngularVelocities(const double w_yaw_);

    void getWheelVelocities(double &v_fl_, double &v_fr_, double &v_rl_, double &v_rr_) const;
    void setWheelVelocities(const double v_fl_, const double v_fr_, const double v_rl_, const double v_rr_);
    void getWheelVelocities(double (&v_wheel_)[4]) const;
    void setWheelVelocities(const double v_wheel_[4]);

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
    double X, Y, yaw; // global 2D position and orientation
    double vx, vy, w_yaw; // vehicle frame velocities
    double ax, ay, a_yaw; // vehicle frame accelerations

    double v_wheel[4];
    double a_wheel[4];

    double Fx, Fy, Mz;
    double Fx_wheel[4];
    double Fy_wheel[4];
    double Fz_wheel[4];

    double steering_angle;
    double motor_torque;
    double wheel_torques[4];

    friend class Vehicle;
};

class TireConfig // tire config and forces
{
public:
    TireConfig();
    TireConfig(double B_, double C_, double D_, double E_, double f_);

    void setTireConfig(const double B_, const double C_, const double D_, const double E_, const double f_);
    void getTireConfig(double &B_, double &C_, double &D_, double &E_, double &f_) const;

    void calcTireForces(const double slipAngle, const double slipRatio, const double Fz_wheel, double &Fx_wheel, double &Fy_wheel);

private:
    double B; // stiffness factor
    double C; // shape factor
    double D; // peak factor
    double E; // curvature factor
    double f; // coeff of relative stiffness for long and lateral directions

    friend class VehicleConfig;
    friend class Vehicle;
};

class VehicleConfig // vehicle parameter configuration
{
public:
    VehicleConfig();
    VehicleConfig(double wheelbase_, double track_width_, double steer_max_,
                  double mass_, double Izz_,
                  double z_cg_, double a_, double r_wheel_, double I_wheel_,
                  double Tmax_, double diff_damping_, TireConfig tire_config_);

    double getMass() const;
    double getIzz() const;
    double getWheelbase() const;
    double getTrackWidth() const;
    double getSteerMax() const;
    double getZcg() const;
    double getA() const;
    double getWheelRadius() const;
    double getTmax() const;
    double getDiffDamping() const;
    double getI_wheel(double &I_wheel_) const;
    TireConfig getTireConfig() const;

private:
    double wheelbase;
    double track_width;
    double steer_max;
    double mass;
    double Izz;
    double z_cg;
    double a;
    double r_wheel;
    double I_wheel;
    double Tmax;
    double diff_damping;

    TireConfig tire_config;

    friend class Vehicle;
};

class Vehicle
{
public:
    Vehicle(const VehicleData &data_, const VehicleConfig &config_, const VehicleInput &input_);
    Vehicle(const VehicleConfig &config_);
    Vehicle();

    void calcMotorTorque();
    void calcSteeringAngle();
    void calcWheelTorques();
    void calcWheelSlipsAndForces();
    void calcTireNormalLoads();
    void calcNetForcesAndMoments();
    void calcBodyAccelerations();
    void calcWheelAccelerations();
    void calcNewState(double dt);
    void stepSimulation(double dt, double steering_input, double throttle_input);

private:
    VehicleData data;
    VehicleConfig config;
    VehicleInput input;
};
