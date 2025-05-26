#include "Vehicle2D.hpp"
#include <cmath> // for std::cos, std::sin, std::atan2, std::sqrt

Vehicle2D::Vehicle2D() : data(Vehicle2DData()), config(Vehicle2DConfig()), input(VehicleInput()) {}
Vehicle2D::Vehicle2D(const Vehicle2DConfig &config_)
    : data(Vehicle2DData()), config(config_), input(VehicleInput()) {}
Vehicle2D::Vehicle2D(const Vehicle2DData &data_, const Vehicle2DConfig &config_, const VehicleInput &input_)
    : data(data_), config(config_), input(input_) {}

void Vehicle2D::calcMotorTorque()
{
    const double Tmax = config.getTmax();
    const double Tnegmax = config.getTnegmax();
    const double Tzero = config.getTzero();
    const double motor_speed = (data.w_wheel[2] + data.w_wheel[3]) / 2.0 * config.getGearRatio();

    // power limit
    const double one_over_abs_motor_speed = 1 / std::abs(motor_speed);
    const double T_Pmax = config.getPmax() * one_over_abs_motor_speed;
    const double T_Pnegmax = config.getPnegmax() * one_over_abs_motor_speed;

    // handle negative throttle
    double motor_torque = 0.0;
    if (std::signbit(input.throttle_input) == std::signbit(motor_speed))
    {
        motor_torque = input.throttle_input * (std::min(Tmax, T_Pmax) + Tzero) - std::copysign(Tzero, motor_speed);
    }
    else
    {
        motor_torque = input.throttle_input * (std::min(Tnegmax, T_Pnegmax) + Tzero) - std::copysign(Tzero, motor_speed);
    }
    data.setMotorTorque(motor_torque);
}

void Vehicle2D::calcBrakeTorque()
{
    double brakeTorque[4];
    double brake_bias = config.getBrakeBias();
    double brake_Tmax = config.getBrakeTmax();
    double halfBrakeTorqueFront = input.brake_input * brake_Tmax * brake_bias / 2.0;
    double halfBrakeTorqueRear = input.brake_input * brake_Tmax * (1.0 - brake_bias) / 2.0;
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double sign[4];
    sign[0] = copysign(1.0, w_wheel[0]) * std::min(1.0, std::abs(w_wheel[0]) * 10);
    sign[1] = copysign(1.0, w_wheel[1]) * std::min(1.0, std::abs(w_wheel[1]) * 10);
    sign[2] = copysign(1.0, w_wheel[2]) * std::min(1.0, std::abs(w_wheel[2]) * 10);
    sign[3] = copysign(1.0, w_wheel[3]) * std::min(1.0, std::abs(w_wheel[3]) * 10);

    brakeTorque[0] = sign[0] * halfBrakeTorqueFront;
    brakeTorque[1] = sign[1] * halfBrakeTorqueFront;
    brakeTorque[2] = sign[2] * halfBrakeTorqueRear;
    brakeTorque[3] = sign[3] * halfBrakeTorqueRear;
    data.setBrakeTorque(brakeTorque);
}

void Vehicle2D::calcSteeringAngle()
{
    data.setSteeringAngle(input.steering_input * config.getSteerMax());
}

void Vehicle2D::calcTractionTorquesRWD()
{
    // get data
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double brake_torque[4];
    data.getBrakeTorque(brake_torque);
    const double diff_damping = config.getDiffDamping();
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate common torques
    const double net_wheel_torque = motor_torque * gear_ratio / 2.0;
    const double damping_torque_rear = diff_damping * (w_wheel[2] - w_wheel[3]);

    // Calculate wheel torques with viscous damping
    double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    wheel_torques[2] = net_wheel_torque - damping_torque_rear;
    wheel_torques[3] = net_wheel_torque + damping_torque_rear;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 4; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D::calcTractionTorques()
{

    if (config.getDrivetrainType() == RWD)
    {
        calcTractionTorquesRWD();
    }
    else if (config.getDrivetrainType() == FWD)
    {
        calcTractionTorquesFWD();
    }
    else if (config.getDrivetrainType() == AWD)
    {
        calcTractionTorquesAWD();
    }
}

void Vehicle2D::calcTractionTorquesFWD()
{
    // get data
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double brake_torque[4];
    data.getBrakeTorque(brake_torque);
    const double diff_damping = config.getDiffDamping();
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate common torques
    const double net_wheel_torque = motor_torque * gear_ratio / 2.0;
    const double damping_torque_front = diff_damping * (w_wheel[0] - w_wheel[1]);

    // Calculate wheel torques with viscous damping
    double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    wheel_torques[0] = net_wheel_torque - damping_torque_front;
    wheel_torques[1] = net_wheel_torque + damping_torque_front;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 4; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D::calcTractionTorquesAWD()
{
    // get data
    double w_wheel[4];
    data.getWheelVelocities(w_wheel);
    double brake_torque[4];
    data.getBrakeTorque(brake_torque);
    const double diff_damping = config.getDiffDamping();
    const double motor_torque = data.getMotorTorque();
    double gear_ratio = config.getGearRatio();

    // calculate common torques
    const double net_wheel_torque = motor_torque * gear_ratio / 4.0;
    const double damping_torque_front = diff_damping * (w_wheel[0] - w_wheel[1]); // damping between front wheels
    const double damping_torque_rear = diff_damping * (w_wheel[2] - w_wheel[3]);  // damping between rear wheels
    const double damping_torque_fr = diff_damping *
                                     (w_wheel[0] + w_wheel[1] - w_wheel[2] - w_wheel[3]) / 4.0 * 0.0; // damping between front and rear wheels

    // Calculate wheel torques with viscous damping
    double wheel_torques[4] = {0.0, 0.0, 0.0, 0.0};
    wheel_torques[0] = net_wheel_torque - damping_torque_front - damping_torque_fr;
    wheel_torques[1] = net_wheel_torque + damping_torque_front - damping_torque_fr;
    wheel_torques[2] = net_wheel_torque - damping_torque_rear + damping_torque_fr;
    wheel_torques[3] = net_wheel_torque + damping_torque_rear + damping_torque_fr;

    // subtract brake torque from wheel torques
    for (int i = 0; i < 4; ++i)
    {
        wheel_torques[i] -= brake_torque[i];
    }
    data.setWheelTorques(wheel_torques);
}

void Vehicle2D::calcTireNormalLoads() // update Fz_wheel array
{
    // get vehicle parameters
    const double g = 9.81;
    const double m = config.getMass();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double h = config.getZcg();
    const double wheelbase = config.getWheelbase();
    const double track_width = config.getTrackWidth();
    double ax, ay;
    data.getLinearAccelerations(ax, ay);

    const double Fz_part = m * g / wheelbase / 2.0;
    const double Fz_front = Fz_part * b;
    const double Fz_rear = Fz_part * a;

    // calc load transfer
    const double dFz_x = (h * m * ax / wheelbase) / 2.0;
    const double dFz_y = (h * m * ay / track_width) / 2.0;

    // calc and set normal loads on each wheel
    double Fz_wheel[4] = {
        Fz_front - dFz_x - dFz_y, // front left
        Fz_front - dFz_x + dFz_y, // front right
        Fz_rear + dFz_x - dFz_y,  // rear left
        Fz_rear + dFz_x + dFz_y   // rear right
    };

    // check for negative normal loads
    int neg_count = 0;
    double Fz_neg = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        if (Fz_wheel[i] < 0.0)
        {
            neg_count++;
            Fz_neg += Fz_wheel[i];
            Fz_wheel[i] = 0.0;
        }
    }
    // if neg count, distribute loads evenly
    if (neg_count > 0)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (Fz_wheel[i] > 0.0)
            {
                Fz_wheel[i] += Fz_neg / (4 - neg_count);
            }
        }
    }

    data.setWheelNormalLoads(Fz_wheel);
}

void Vehicle2D::calcWheelSlipsAndForces()
{
    // get vehicle states
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double b = config.getWheelbase() - a;
    const double half_track_width = config.getTrackWidth() / 2.0;
    const double r_wheel = config.getWheelRadius();

    double w_wheel[4];
    double Fz_wheel[4];
    double vx, vy, w_yaw;
    data.getWheelVelocities(w_wheel);
    data.getWheelNormalLoads(Fz_wheel);
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);

    // calculate velocities at wheel centers
    const double vx_yaw = half_track_width * w_yaw;
    double vxw[4] = {
        vx - vx_yaw,  // fl
        vx + vx_yaw,  // fr
        vx - vx_yaw,  // rl
        vx + vx_yaw}; // rr
    double vyw[4] = {
        vy + a * w_yaw,
        vy + a * w_yaw,
        vy - b * w_yaw,
        vy - b * w_yaw};

    // resolve x and y velocities in wheel frame for wheels that steer
    for (int i = 0; i < 2; ++i)
    {
        const double vxw_temp = vxw[i];
        vxw[i] = vxw_temp * std::cos(steering_angle) + vyw[i] * std::sin(steering_angle);
        vyw[i] = vyw[i] * std::cos(steering_angle) - vxw_temp * std::sin(steering_angle);
    }

    // calculate slip angles
    double slip_angle[4];
    double slip_ratio[4];
    slip_angle[0] = std::atan2(vyw[0], vxw[0]); // front left
    slip_angle[1] = std::atan2(vyw[1], vxw[1]); // front right
    slip_angle[2] = std::atan2(vyw[2], vxw[2]); // rear left
    slip_angle[3] = std::atan2(vyw[3], vxw[3]); // rear right

    // calculate tire forces
    double Fx_wheel[4];
    double Fy_wheel[4];
    const double slip_threshold = 0.5; // threshold for slip ratio
    for (int i = 0; i < 4; ++i)
    {
        // calc slip ratios
        double denominator = std::max(std::max(std::abs(w_wheel[i] * r_wheel), std::abs(vxw[i])), slip_threshold);
        slip_ratio[i] = (w_wheel[i] * r_wheel - vxw[i]) / denominator;

        // calculate tire forces
        config.getTireConfig().calcTireForces(RAD2DEG(slip_angle[i]), slip_ratio[i], w_wheel[i], Fz_wheel[i], Fx_wheel[i], Fy_wheel[i]);
    }
    data.setWheelForces(Fx_wheel, Fy_wheel);
}

void Vehicle2D::calcAerodynamicForces(double (&F)[3], double (&M)[3])
{
    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);

    F[0] = -0.5 * config.getCDx() * config.getRho() * config.getFrontalArea() * vx * vx;
    F[1] = 0.0;
    F[2] = 0.0;
    M[0] = 0.0;
    M[1] = 0.0;
    M[2] = 0.0;
    M[3] = -0.5 * config.getCMz() * config.getRho() * config.getFrontalArea() * w_yaw * w_yaw;
}

void Vehicle2D::calcNetForcesAndMoments() // update Fx, Fy, Mz
{
    // calculate net forces and moments acting on the vehicle
    const double steering_angle = data.getSteeringAngle();
    const double a = config.getA();
    const double track_width = config.getTrackWidth();
    const double wheelbase = config.getWheelbase();
    const double b = wheelbase - a;
    const double half_track_width = track_width / 2.0;
    double Fx_body[4];
    double Fy_body[4];
    double Fx_wheel[4];
    double Fy_wheel[4];
    data.getWheelForces(Fx_wheel, Fy_wheel);

    // aerodynamic forces
    double F_aero[3];
    double M_aero[3];
    calcAerodynamicForces(F_aero, M_aero);

    double Fx = F_aero[0];
    double Fy = F_aero[1];
    double Mz = M_aero[3];

    // resolve forces in vehicle frame for wheels that steer
    for (int i = 0; i < 2; ++i)
    {
        double Fx_temp = Fx_wheel[i];
        Fx_body[i] = Fx_temp * std::cos(steering_angle) - Fy_wheel[i] * std::sin(steering_angle);
        Fy_body[i] = Fx_temp * std::sin(steering_angle) + Fy_wheel[i] * std::cos(steering_angle);
        Fx += Fx_body[i];
        Fy += Fy_body[i];
    }
    // resolve forces in vehicle frame for wheels that do not steer
    for (int i = 2; i < 4; ++i)
    {
        Fx_body[i] = Fx_wheel[i];
        Fy_body[i] = Fy_wheel[i];
        Fx += Fx_body[i];
        Fy += Fy_body[i];
    }
    // calculate moments about the center of mass
    Mz += -Fx_body[0] * half_track_width + Fy_body[0] * a; // front left
    Mz += Fx_body[1] * half_track_width + Fy_body[1] * a;  // front right
    Mz += -Fx_body[2] * half_track_width - Fy_body[2] * b; // rear left
    Mz += Fx_body[3] * half_track_width - Fy_body[3] * b;  // rear right

    data.setBodyForcesAndMoments(Fx, Fy, Mz);
}

void Vehicle2D::calcBodyAccelerations() // update resultant body accelerations ax, ay, a_yaw
{
    // calculate resultant body accelerations based on net forces and moments
    double ax, ay, a_yaw;
    const double mass = config.getMass();
    const double Izz = config.getIzz();
    double Fx, Fy, Mz;
    data.getBodyForcesAndMoments(Fx, Fy, Mz);

    double vx, vy, w_yaw;
    data.getLinearVelocities(vx, vy);
    data.getAngularVelocities(w_yaw);
    // double vehicle_slip_angle = std::atan2(vy, vx);

    ax = Fx / mass;
    ay = Fy / mass;
    a_yaw = Mz / Izz;

    data.setLinearAccelerations(ax, ay);
    data.setAngularAccelerations(a_yaw);
}

void Vehicle2D::calcWheelAccelerations()
{
    double a_wheel[4];
    double I_wheel = config.getI_wheel();
    double wheel_radius = config.getWheelRadius();

    double wheel_torques[4];
    double Fx_wheel[4];
    double Fy_wheel[4];
    data.getWheelTorques(wheel_torques);
    data.getWheelForces(Fx_wheel, Fy_wheel);

    for (int i = 0; i < 4; ++i)
    {
        a_wheel[i] = (wheel_torques[i] - Fx_wheel[i] * wheel_radius) / I_wheel;
    }
    data.setWheelAccelerations(a_wheel);
}

void Vehicle2D::calcNewState(double dt) // update vehicle state (X, Y, yaw, vx, vy, w_yaw)
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

    // calculate new velocities
    vx += (ax + w_yaw * vy) * dt;
    vy += (ay - w_yaw * vx) * dt;
    w_yaw += a_yaw * dt;

    // calculate new position and orientation
    double sin_yaw = std::sin(yaw);
    double cos_yaw = std::cos(yaw);
    X += (vx * cos_yaw - vy * sin_yaw) * dt;
    Y += (vx * sin_yaw + vy * cos_yaw) * dt;
    yaw += w_yaw * dt;
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));

    // calculate new wheel velocities
    double w_wheel[4];
    double a_wheel[4];
    double p_wheel[4];
    data.getWheelVelocities(w_wheel);
    data.getWheelAccelerations(a_wheel);
    data.getWheelRotations(p_wheel);
    for (int i = 0; i < 4; ++i)
    {
        w_wheel[i] += a_wheel[i] * dt;
    }
    for (int i = 0; i < 4; ++i)
    {
        p_wheel[i] += w_wheel[i] * dt + 0.5 * a_wheel[i] * dt * dt;
    }

    // set
    data.setPosition(X, Y);
    data.setOrientation(yaw);
    data.setLinearVelocities(vx, vy);
    data.setAngularVelocities(w_yaw);
    data.setWheelVelocities(w_wheel);
    data.setWheelRotations(p_wheel);
}

bool Vehicle2D::stepSimulation(double dt, double steering_input, double throttle_input, double brake_input)
{
    input.setSteeringInput(steering_input);
    input.setThrottleInput(throttle_input);
    input.setBrakeInput(brake_input);
    calcSteeringAngle();
    calcMotorTorque();
    calcBrakeTorque();
    calcTractionTorques();
    calcTireNormalLoads();
    calcWheelSlipsAndForces();
    calcNetForcesAndMoments();
    calcBodyAccelerations();
    calcWheelAccelerations();
    calcNewState(dt);

    return true;
}
Vehicle2DData &Vehicle2D::getVehicle2DData() { return data; }
Vehicle2DConfig &Vehicle2D::getVehicle2DConfig() { return config; }
VehicleInput &Vehicle2D::getVehicleInput() { return input; }
