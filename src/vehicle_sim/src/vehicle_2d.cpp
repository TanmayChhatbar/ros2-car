#pragma once

#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

class VehicleInput // from user/controller
{
public:
    // constructor
    VehicleInput()
        : steering_input(0.0), throttle_input(0.0) {}

    // getters and setters for steering angle and motor torque
    void getSteeringInput(double &steering_input_) const
    {
        steering_input_ = steering_input;
    }
    void setSteeringInput(double steering_input_)
    {
        if (steering_input_ < -1.0)
            steering_input = -1.0;
        else if (steering_input_ > 1.0)
            steering_input = 1.0;
        else
            steering_input = steering_input_;
    }
    void getThrottleInput(double &throttle_input_) const
    {
        throttle_input_ = throttle_input;
    }
    void setThrottleInput(double throttle_input_)
    {
        if (throttle_input_ < -1.0)
            throttle_input = -1.0;
        else if (throttle_input_ > 1.0)
            throttle_input = 1.0;
        else
            throttle_input = throttle_input_;
    }

private:
    double steering_input; // [rad] steering angle
    double throttle_input; // [0, 1] throttle_input input

    friend class Vehicle; // allow Vehicle to access private members
};

class VehicleData
{
public:
    // constructor
    VehicleData()
        : X(0.0), Y(0.0), yaw(0.0),
          vx(0.0), vy(0.0), w_yaw(0.0),
          ax(0.0), ay(0.0), a_yaw(0.0),
          v_wheel{0.0, 0.0, 0.0, 0.0},
          a_wheel{0.0, 0.0, 0.0, 0.0},
          Fx(0.0), Fy(0.0), Mz(0.0),
          Fx_wheel{0.0, 0.0, 0.0, 0.0},
          Fy_wheel{0.0, 0.0, 0.0, 0.0},
          Fz_wheel{0.0, 0.0, 0.0, 0.0},
          steering_angle(0.0), motor_torque(0.0),
          wheel_torques{0.0, 0.0, 0.0, 0.0} {}

    // getters and setters for inputs
    double getSteeringAngle() const { return steering_angle; }
    void setSteeringAngle(const double steering_angle_) { steering_angle = steering_angle_; }
    double getMotorTorque() const { return motor_torque; }
    void setMotorTorque(const double motor_torque_) { motor_torque = motor_torque_; }

    // getters and setters for position and orientation
    void getPosition(double &x_, double &y_) const
    {
        x_ = X;
        y_ = Y;
    }
    void setPosition(const double x_, const double y_)
    {
        X = x_;
        Y = y_;
    }
    void getOrientation(double &yaw_) const { yaw_ = yaw; }
    void setOrientation(const double yaw_) { yaw = yaw_; }

    // getters and setters for velocities
    void getLinearVelocities(double &vx_, double &vy_) const
    {
        vx_ = vx;
        vy_ = vy;
    }
    void setLinearVelocities(const double vx_, const double vy_)
    {
        vx = vx_;
        vy = vy_;
    }
    void getAngularVelocities(double &w_yaw_) const { w_yaw_ = w_yaw; }
    void setAngularVelocities(const double w_yaw_) { w_yaw = w_yaw_; }

    void getWheelVelocities(double &v_fl_, double &v_fr_, double &v_rl_, double &v_rr_) const
    {
        v_fl_ = v_wheel[0];
        v_fr_ = v_wheel[1];
        v_rl_ = v_wheel[2];
        v_rr_ = v_wheel[3];
    }
    void setWheelVelocities(const double v_fl_, const double v_fr_, const double v_rl_, const double v_rr_)
    {
        v_wheel[0] = v_fl_;
        v_wheel[1] = v_fr_;
        v_wheel[2] = v_rl_;
        v_wheel[3] = v_rr_;
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
    void getLinearAccelerations(double &ax_, double &ay_) const
    {
        ax_ = ax;
        ay_ = ay;
    }
    void setLinearAccelerations(const double ax_, const double ay_)
    {
        ax = ax_;
        ay = ay_;
    }
    void getAngularAccelerations(double &a_yaw_) const { a_yaw_ = a_yaw; }
    void setAngularAccelerations(const double a_yaw_) { a_yaw = a_yaw_; }
    void getWheelAccelerations(double &a_fl_, double &a_fr_, double &a_rl_, double &a_rr_) const
    {
        a_fl_ = a_wheel[0];
        a_fr_ = a_wheel[1];
        a_rl_ = a_wheel[2];
        a_rr_ = a_wheel[3];
    }
    void setWheelAccelerations(const double a_fl_, const double a_fr_, const double a_rl_, const double a_rr_)
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

    // getters and setters for forces and moments
    void getBodyForcesAndMoments(double &Fx_, double &Fy_, double &Mz_) const
    {
        Fx_ = Fx;
        Fy_ = Fy;
        Mz_ = Mz;
    }
    void setBodyForcesAndMoments(const double Fx_, const double Fy_, const double Mz_)
    {
        Fx = Fx_;
        Fy = Fy_;
        Mz = Mz_;
    }
    void getWheelNormalLoads(double (&Fz_wheel_)[4]) const
    {
        Fz_wheel_[0] = Fz_wheel[0];
        Fz_wheel_[1] = Fz_wheel[1];
        Fz_wheel_[2] = Fz_wheel[2];
        Fz_wheel_[3] = Fz_wheel[3];
    }
    void setWheelNormalLoads(const double Fz_wheel_[4])
    {
        Fz_wheel[0] = Fz_wheel_[0];
        Fz_wheel[1] = Fz_wheel_[1];
        Fz_wheel[2] = Fz_wheel_[2];
        Fz_wheel[3] = Fz_wheel_[3];
    }
    void getWheelForces(double (&Fx_wheel_)[4], double (&Fy_wheel_)[4]) const
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
    void setWheelForces(const double Fx_wheel_[4], const double Fy_wheel_[4])
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
    void getWheelTorques(double (&wheel_torques_)[4]) const
    {
        wheel_torques_[0] = wheel_torques[0];
        wheel_torques_[1] = wheel_torques[1];
        wheel_torques_[2] = wheel_torques[2];
        wheel_torques_[3] = wheel_torques[3];
    }
    void setWheelTorques(const double (&wheel_torques_)[4])
    {
        wheel_torques[0] = wheel_torques_[0];
        wheel_torques[1] = wheel_torques_[1];
        wheel_torques[2] = wheel_torques_[2];
        wheel_torques[3] = wheel_torques_[3];
    }

private:
    double X, Y, yaw;     // global 2D position and orientation
    double vx, vy, w_yaw; // vehicle frame velocities
    double ax, ay, a_yaw; // vehicle frame accelerations

    // wheel velocities and accelerations
    double v_wheel[4];
    double a_wheel[4];

    // forces
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
    // pacejka tire model
public:
    // constructor
    TireConfig()
        : B(0.0), C(0.0), D(0.0), E(0.0), f(1.0) {}
    TireConfig(double B_, double C_, double D_, double E_, double f_)
        : B(B_), C(C_), D(D_), E(E_), f(f_) {}

    // tire forces
    void setTireConfig(const double B_, const double C_, const double D_, const double E_, const double f_)
    {
        B = B_;
        C = C_;
        D = D_;
        E = E_;
        f = f_;
    }
    void getTireConfig(double &B_, double &C_, double &D_, double &E_, double &f_) const
    {
        B_ = B;
        C_ = C;
        D_ = D;
        E_ = E;
        f_ = f;
    }

    void calcTireForces(const double slipAngle, const double slipRatio, const double Fz_wheel, double &Fx_wheel, double &Fy_wheel)
    {
        // calculate tire forces using pacejka model
        double slipNet = std::sqrt(slipAngle * slipAngle + slipRatio * slipRatio * f * f);

        // pacejka tire formula
        if (slipNet == 0.0)
        {
            Fx_wheel = 0.0;
            Fy_wheel = 0.0;
            return;
        }
        double Fnet = D * std::sin(C * std::atan(B * slipNet - E * (B * slipNet - std::atan(B * slipNet))));
        Fx_wheel = Fnet * slipRatio * f / slipNet;
        Fy_wheel = Fnet * slipAngle / slipNet;
    }

private:
    double B; // stiffness factor
    double C; // shape factor
    double D; // peak factor
    double E; // curvature factor
    double f; // coeff of relative stiffness for long and lateral directions

    friend class VehicleConfig; // allow VehicleConfig to access private members
    friend class Vehicle;       // allow Vehicle to access private members
};

class VehicleConfig // vehicle parameter configuration
{
    VehicleConfig()
        : wheelbase(0.0), track_width(0.0), steer_max(0.0),
          mass(0.0), Izz(0.0), z_cg(0.0), a(0.0), r_wheel(0.0),
          I_wheel(0.0), Tmax(0.0), diff_damping(0.0), tire_config() {}
    VehicleConfig(double wheelbase_, double track_width_, double steer_max_,
                  double mass_, double Izz_,
                  double z_cg_, double a_, double r_wheel_, double I_wheel_,
                  double Tmax_, double diff_damping_, TireConfig tire_config_)
        : wheelbase(wheelbase_), track_width(track_width_), steer_max(steer_max_),
          mass(mass_), Izz(Izz_), z_cg(z_cg_), a(a_), r_wheel(r_wheel_),
          I_wheel(I_wheel_), Tmax(Tmax_), diff_damping(diff_damping_),
          tire_config(tire_config_) {}

    double getMass() const { return mass; }
    double getIzz() const { return Izz; }
    double getWheelbase() const { return wheelbase; }
    double getTrackWidth() const { return track_width; }
    double getSteerMax() const { return steer_max; }
    double getZcg() const { return z_cg; }
    double getA() const { return a; }
    double getWheelRadius() const { return r_wheel; }
    double getTmax() const { return Tmax; }
    double getDiffDamping() const { return diff_damping; }
    double getI_wheel(double &I_wheel_) const { I_wheel_ = I_wheel; }
    TireConfig getTireConfig() const { return tire_config; }

private:
    double wheelbase;   // [m] distance between front and rear axles
    double track_width; // [m] distance between left and right wheels
    double steer_max;   // [rad] maximum steering angle
    double mass;        // [kg] vehicle mass
    double Izz;         // [kgm2] moment of inertia about the z-axis
    double z_cg;        // [m] static center of gravity height from ground
    double a;           // [m] distance from front axle to center of gravity
    double r_wheel;     // [m] wheel radius
    double I_wheel;     // [kgm2] moment of inertia of wheel
    double Tmax;
    double diff_damping; // [Nms/rad] viscous damping of the differential

    TireConfig tire_config; // tire parameters

    friend class Vehicle;
};

class Vehicle
{
public:
    Vehicle(const VehicleData &data_, const VehicleConfig &config_, const VehicleInput &input_)
        : data(data_), config(config_, input(input_)) {}
    Vehicle(const VehicleConfig &config_)
        : data(VehicleData()), config(config_), input(VehicleInput()) {}
    Vehicle() : data(VehicleData()), config(VehicleConfig(), input(VehicleInput())) {}

    void calcMotorTorque()
    {
        data.setMotorTorque(input.throttle_input * config.getTmax());
    }
    void calcSteeringAngle()
    {
        data.setSteeringAngle(input.steering_input * config.getSteerMax());
    }
    void calcWheelTorques()
    {
        double v_wheel[4];
        data.getWheelVelocities(v_wheel);
        double diff_damping = config.getDiffDamping();
        double motor_torque = data.getMotorTorque();     // Use getter for motor torque
        double dv_wheel_front = v_wheel[0] - v_wheel[1]; // front left - front right
        double dv_wheel_rear = v_wheel[2] - v_wheel[3];
        double damping_front = diff_damping * dv_wheel_front;
        double damping_rear = diff_damping * dv_wheel_rear;

        // Calculate wheel torques with viscous damping
        double wheel_torques[4];
        wheel_torques[0] = motor_torque / 4.0 - damping_front;
        wheel_torques[1] = motor_torque / 4.0 + damping_front;
        wheel_torques[2] = motor_torque / 4.0 - damping_rear;
        wheel_torques[3] = motor_torque / 4.0 + damping_rear;
        data.setWheelTorques(wheel_torques);
    }

    void calcWheelSlipsAndForces()
    {

        // get vehicle states
        double vx, vy, w_yaw;
        data.getLinearVelocities(vx, vy);
        data.getAngularVelocities(w_yaw);
        double steering_input = data.getSteeringAngle();
        double a = config.getA();
        double b = config.getWheelbase() - a;
        double half_track_width = config.getTrackWidth() / 2.0;
        double r_wheel = config.getWheelRadius();
        double v_wheel[4];
        data.getWheelVelocities(v_wheel);
        double Fz_wheel[4];
        data.getWheelNormalLoads(Fz_wheel);

        // calculate slips and forces
        double slipAngle[4];
        double slipRatio[4];
        double vx_yaw = half_track_width * w_yaw;
        double vxw[4] = {
            vx - vx_yaw,
            vx + vx_yaw,
            vx - vx_yaw,
            vx + vx_yaw};
        double vyw[4] = {
            vy + a * w_yaw,
            vy + a * w_yaw,
            vy - b * w_yaw,
            vy - b * w_yaw};
        slipAngle[0] = std::atan2(vyw[0], vxw[0]) - steering_input; // front left
        slipAngle[1] = std::atan2(vyw[1], vxw[1]) - steering_input; // front right
        slipAngle[2] = std::atan2(vyw[2], vxw[2]);                  // rear left
        slipAngle[3] = std::atan2(vyw[3], vxw[3]);                  // rear right

        double Fx_wheel[4];
        double Fy_wheel[4];
        for (int i = 0; i < 4; ++i)
        {
            // calc net slip
            // TODO: slip ratio switch for positive and negative slip
            if (std::abs(vxw[i]) > 1e-6)
            {
                slipRatio[i] = v_wheel[i] * r_wheel / vxw[i] - 1;
            }
            else
            {
                slipRatio[i] = 0.0;
            }
            // calculate tire forces
            config.getTireConfig().calcTireForces(slipAngle[i], slipRatio[i], Fz_wheel[i], Fx_wheel[i], Fy_wheel[i]);
        }
        data.setWheelForces(Fx_wheel, Fy_wheel);
    }

    void calcTireNormalLoads() // update Fz_wheel array
    {
        // get vehicle parameters
        double m = config.getMass();
        double g = 9.81; // gravity
        double a = config.getA();
        double b = config.getWheelbase() - a;
        double h = config.getZcg();
        double half_track_width = config.getTrackWidth() / 2.0;
        double wheelbase = config.getWheelbase();
        double ax, ay;
        data.getLinearAccelerations(ax, ay);
        double Fz_front = m * g * b / (a + b) / 2.0;
        double Fz_rear = m * g * a / (a + b) / 2.0;

        // calc load transfer
        double dFz_x = h * m * ax / wheelbase;
        double dFz_y = h * m * ay / half_track_width;

        // set normal loads on each wheel
        double Fz_wheel[4] = {
            Fz_front - dFz_x / 2.0 - dFz_y / 2.0, // front left
            Fz_front - dFz_x / 2.0 + dFz_y / 2.0, // front right
            Fz_rear + dFz_x / 2.0 - dFz_y / 2.0,  // rear left
            Fz_rear + dFz_x / 2.0 + dFz_y / 2.0   // rear right
        };
        data.setWheelNormalLoads(Fz_wheel);
    }
    void calcNetForcesAndMoments() // update Fx, Fy, Mz
    {
        // calculate net forces and moments acting on the vehicle
        double Fx = 0.0;
        double Fy = 0.0;
        double Mz = 0.0;
        double steering_angle = data.getSteeringAngle();
        double a = config.getA();
        double track_width = config.getTrackWidth();
        double wheelbase = config.getWheelbase();
        double half_track_width = track_width / 2.0;
        double Fx_wheel[4];
        double Fy_wheel[4];
        data.getWheelForces(Fx_wheel, Fy_wheel);

        for (int i = 0; i < 2; i++)
        {
            Fx += Fx_wheel[i] * std::cos(steering_angle) - Fy_wheel[i] * std::sin(steering_angle);
            Fy += Fy_wheel[i] * std::sin(steering_angle) + Fx_wheel[i] * std::cos(steering_angle);
        }
        for (int i = 2; i < 4; i++)
        {
            Fx += Fx_wheel[i];
            Fy += Fy_wheel[i];
        }
        Mz += -Fx_wheel[0] * half_track_width + Fy_wheel[0] * a;
        Mz += -Fx_wheel[1] * half_track_width + Fy_wheel[1] * a;
        Mz += -Fx_wheel[2] * half_track_width - Fy_wheel[2] * (wheelbase - a);
        Mz += -Fx_wheel[3] * half_track_width - Fy_wheel[3] * (wheelbase - a);
        data.setBodyForcesAndMoments(Fx, Fy, Mz);
    }
    void calcBodyAccelerations() // update resultant body accelerations ax, ay, a_yaw
    {
        double ax, ay, a_yaw, Fx, Fy, Mz;
        double mass = config.getMass();
        double Izz = config.getIzz();
        data.getBodyForcesAndMoments(Fx, Fy, Mz);

        // calculate resultant body accelerations based on net forces and moments
        ax = Fx / mass;
        ay = Fy / mass;
        a_yaw = Mz / Izz;

        data.setLinearAccelerations(ax, ay);
        data.setAngularAccelerations(a_yaw);
    }
    void calcWheelAccelerations()
    {
        double a_wheel[4];
        double I_wheel;
        config.getI_wheel(I_wheel);
        double wheel_radius = config.getWheelRadius();
        double wheel_torques[4];
        data.getWheelTorques(wheel_torques);
        double Fx_wheel[4];
        double Fy_wheel[4];
        data.getWheelForces(Fx_wheel, Fy_wheel);
        for (int i = 0; i < 4; ++i)
        {
            a_wheel[i] = (wheel_torques[i] - Fx_wheel[i] * wheel_radius) / I_wheel;
        }
        data.setWheelAccelerations(a_wheel);
    }
    void calcNewState(double dt) // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
    {
        double vx, vy, w_yaw;
        data.getLinearVelocities(vx, vy);
        data.getAngularVelocities(w_yaw);

        double ax, ay, a_yaw;
        data.getLinearAccelerations(ax, ay);
        data.getAngularAccelerations(a_yaw);

        double X, Y, yaw;
        data.getPosition(X, Y);
        data.getOrientation(yaw);

        vx += (ax - w_yaw * vy) * dt;
        vy += (ay + w_yaw * vx) * dt;
        w_yaw += a_yaw * dt;

        X += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
        Y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
        yaw += w_yaw * dt + 0.5 * a_yaw * dt * dt;

        double v_wheel[4];
        double a_wheel[4];
        data.getWheelVelocities(v_wheel);
        data.getWheelAccelerations(a_wheel);
        for (int i = 0; i < 4; ++i)
        {
            v_wheel[i] += a_wheel[i] * dt;
        }

        // set
        data.setPosition(X, Y);
        data.setOrientation(yaw);
        data.setLinearVelocities(vx, vy);
        data.setAngularVelocities(w_yaw);
        data.setLinearAccelerations(ax, ay);
        data.setAngularAccelerations(a_yaw);
        data.setWheelVelocities(v_wheel);
        data.setWheelAccelerations(a_wheel);
    }
    void stepSimulation(double dt, double steering_input, double throttle_input)
    {
        input.setSteeringInput(steering_input);
        input.setThrottleInput(throttle_input);
        calcSteeringAngle();
        calcMotorTorque();
        calcWheelSlipsAndForces();
        calcTireNormalLoads();
        calcNetForcesAndMoments();
        calcBodyAccelerations();
        calcWheelAccelerations();
        calcNewState(dt);
    }

private:
    VehicleData data;
    VehicleConfig config;
    VehicleInput input;
};
