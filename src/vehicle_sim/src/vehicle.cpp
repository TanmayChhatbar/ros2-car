#pragma once

#include <cmath>    // for mathematical constants if needed
#include <iostream> // for printing

// vehicle state data
class VehicleData
{
public:
    // constructor
    VehicleData()
        : X(0.0), Y(0.0), Z(0.0), roll(0.0), pitch(0.0), yaw(0.0),
          vx(0.0), vy(0.0), vz(0.0), w_roll(0.0), w_pitch(0.0), w_yaw(0.0),
          ax(0.0), ay(0.0), az(0.0), a_roll(0.0), a_pitch(0.0), a_yaw(0.0),
          v_wheel{0.0, 0.0, 0.0, 0.0}, a_wheel{0.0, 0.0, 0.0, 0.0},
          Fx(0.0), Fy(0.0), Mz(0.0),
          Fx_wheel{0.0, 0.0, 0.0, 0.0}, Fy_wheel{0.0, 0.0, 0.0, 0.0}, Fz_wheel{0.0, 0.0, 0.0, 0.0} {}

    // getters and setters for position
    void getPosition(double &x_, double &y_, double &z_) const
    {
        x_ = X;
        y_ = Y;
        z_ = Z;
    }
    void setPosition(double x_, double y_, double z_)
    {
        X = x_;
        Y = y_;
        Z = z_;
    }
    void getOrientation(double &roll_, double &pitch_, double &yaw_) const
    {
        roll_ = roll;
        pitch_ = pitch;
        yaw_ = yaw;
    }
    void setOrientation(double roll_, double pitch_, double yaw_)
    {
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }

    // getters and setters for velocities
    void getLinearVelocities(double &vx_, double &vy_, double &vz_) const
    {
        vx_ = vx;
        vy_ = vy;
        vz_ = vz;
    }
    void setLinearVelocities(double vx_, double vy_, double vz_)
    {
        vx = vx_;
        vy = vy_;
        vz = vz_;
    }
    void getAngularVelocities(double &w_roll_, double &w_pitch_, double &w_yaw_) const
    {
        w_roll_ = w_roll;
        w_pitch_ = w_pitch;
        w_yaw_ = w_yaw;
    }
    void setAngularVelocities(double w_roll_, double w_pitch_, double w_yaw_)
    {
        w_roll = w_roll_;
        w_pitch = w_pitch_;
        w_yaw = w_yaw_;
    }
    void getWheelVelocities(double &v_fl_, double &v_fr_, double &v_rl_, double &v_rr_)
    {
        v_fl_ = v_wheel[0];
        v_fr_ = v_wheel[1];
        v_rl_ = v_wheel[2];
        v_rr_ = v_wheel[3];
    }
    void setWheelVelocities(double fl_, double fr_, double rl_, double rr_)
    {
        v_wheel[0] = fl_;
        v_wheel[1] = fr_;
        v_wheel[2] = rl_;
        v_wheel[3] = rr_;
    }
    void getWheelVelocities(double (&v_wheel_)[4]) const
    {
        for (int i = 0; i < 4; ++i)
        {
            v_wheel_[i] = v_wheel[i];
        }
    }
    void setWheelVelocities(const double v_wheel_[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            v_wheel[i] = v_wheel_[i];
        }
    }

    // getters and setters for accelerations
    void getLinearAccelerations(double &ax_, double &ay_, double &az_) const
    {
        ax_ = ax;
        ay_ = ay;
        az_ = az;
    }
    void setLinearAccelerations(double ax_, double ay_, double az_)
    {
        ax = ax_;
        ay = ay_;
        az = az_;
    }
    void getAngularAccelerations(double &a_roll_, double &a_pitch_, double &a_yaw_) const
    {
        a_roll_ = a_roll;
        a_pitch_ = a_pitch;
        a_yaw_ = a_yaw;
    }
    void setAngularAccelerations(double w_roll_, double w_pitch_, double w_yaw_)
    {
        w_roll = w_roll_;
        w_pitch = w_pitch_;
        w_yaw = w_yaw_;
    }
    void getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_)
    {
        a_fl_ = a_wheel[0];
        a_fr_ = a_wheel[1];
        a_rl_ = a_wheel[2];
        a_rr_ = a_wheel[3];
    }
    void setWheelAccelerations(double a_fl_, double a_fr_, double a_rl_, double a_rr_)
    {
        a_wheel[0] = a_fl_;
        a_wheel[1] = a_fr_;
        a_wheel[2] = a_rl_;
        a_wheel[3] = a_rr_;
    }
    void getWheelAccelerations(double (&a_wheel_)[4]) const
    {
        for (int i = 0; i < 4; ++i)
        {
            a_wheel_[i] = a_wheel[i];
        }
    }
    void setWheelAccelerations(const double a_wheel_[4])
    {
        for (int i = 0; i < 4; ++i)
        {
            a_wheel[i] = a_wheel_[i];
        }
    }

private:
    // global 3D position
    double X, Y, Z;

    // orientation (roll, pitch, yaw)
    double roll, pitch, yaw;

    // vehicle frame velocities
    double vx, vy, vz;
    double w_roll, w_pitch, w_yaw;

    // vehicle frame accelerations
    double ax, ay, az;
    double a_roll, a_pitch, a_yaw;

    // wheel velocities and accelerations
    double v_wheel[4];
    double a_wheel[4];
    
    // forces
    double Fx, Fy, Mz;
    double Fx_wheel[4];
    double Fy_wheel[4];
    double Fz_wheel[4];

    friend class Vehicle;
};

// pacejka tire model
class TireConfig
{
public:
    // constructor
    TireConfig()
        : B(0.0), C(0.0), D(0.0), E(0.0), f(0.0) {}
    TireConfig(double B, double C, double D, double E, double f)
        : B(B), C(C), D(D), E(E), f(f) {}

    double setTireConfig(double B_, double C_, double D_, double E_, double f_)
    {
        B = B_;
        C = C_;
        D = D_;
        E = E_;
        f = f_;
    }

private:
    double B; // stiffness factor
    double C; // shape factor
    double D; // peak factor
    double E; // curvature factor
    friend class VehicleConfig; // allow VehicleConfig to access private members
    friend class Vehicle;      // allow Vehicle to access private members
};

// vehicle configuration
class VehicleConfig
{
public:
    VehicleConfig()
        : wheelbase(0.0), track_width(0.0), steer_max(0.0),
          mass(0.0), Ixx(0.0), Iyy(0.0), Izz(0.0), z_cg(0.0), a(0.0), tire_config() {}
    VehicleConfig(double wheelbase_, double track_width_, double steer_max_,
                  double mass_, double Ixx_, double Iyy_, double Izz_,
                  double z_cg_, double a_, TireConfig tire_config_)
        : wheelbase(wheelbase_), track_width(track_width_), steer_max(steer_max_),
          mass(mass_), Ixx(Ixx_), Iyy(Iyy_), Izz(Izz_), z_cg(z_cg_), a(a_), tire_config(tire_config_) {}

private:
    double wheelbase;   // [m] distance between front and rear axles
    double track_width; // [m] distance between left and right wheels
    double steer_max;   // [rad] maximum steering angle
    double mass;        // [kg] vehicle mass
    double Ixx;         // [kgm2] moment of inertia about the x-axis
    double Iyy;         // [kgm2] moment of inertia about the y-axis
    double Izz;         // [kgm2] moment of inertia about the z-axis
    double z_cg;        // [m] static center of gravity height from ground
    double a;           // [m] distance from front axle to center of gravity

    TireConfig tire_config; // tire parameters

    friend class Vehicle;
};

class Vehicle
{
public:
    Vehicle(const VehicleData &data_, const VehicleConfig &config_)
        : data(data_), config(config_) {}

    void calcTireForces(double &Fx, double &Fy)
    {
    // Calculate forward slip velocity

    // Calculate forward force

    // Calculate lateral slip velocity

    // Calculate lateral force

    }

private:
    VehicleData data;
    VehicleConfig config;
};
