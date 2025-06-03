#include "Vehicle2D.hpp"

#include <iostream>
#include <fstream>

// cSV header and data formatting
void writeCSVHeader(std::ofstream &file)
{
     file << "time,X,Y,yaw,vx,vy,w_yaw,ax,ay,a_yaw,"
          << "w_wheel_fl,w_wheel_fr,w_wheel_rl,w_wheel_rr,"
          << "Fx_wheel_fl,Fx_wheel_fr,Fx_wheel_rl,Fx_wheel_rr,"
          << "Fy_wheel_fl,Fy_wheel_fr,Fy_wheel_rl,Fy_wheel_rr,"
          << "steering,motor_torque" << std::endl;
}

void writeCSVData(std::ofstream &file, double time, const Vehicle2DData &data,
                  const VehicleInput &input)
{
     double X, Y, yaw;
     double vx, vy, w_yaw;
     double ax, ay, a_yaw;
     double w_wheel[4];
     double Fx_wheel[4], Fy_wheel[4];
     double steering_angle = data.getSteeringAngle();
     double motor_torque = data.getMotorTorque();

     data.getPosition(X, Y);
     data.getOrientation(yaw);
     data.getLinearVelocities(vx, vy);
     data.getAngularVelocities(w_yaw);
     data.getLinearAccelerations(ax, ay);
     data.getAngularAccelerations(a_yaw);
     data.getWheelVelocities(w_wheel);
     data.getWheelForces(Fx_wheel, Fy_wheel);

     file << time << ","
          << X << "," << Y << "," << yaw << ","
          << vx << "," << vy << "," << w_yaw << ","
          << ax << "," << ay << "," << a_yaw << ","
          << w_wheel[0] << "," << w_wheel[1] << "," << w_wheel[2] << "," << w_wheel[3] << ","
          << Fx_wheel[0] << "," << Fx_wheel[1] << "," << Fx_wheel[2] << "," << Fx_wheel[3] << ","
          << Fy_wheel[0] << "," << Fy_wheel[1] << "," << Fy_wheel[2] << "," << Fy_wheel[3] << ","
          << steering_angle << "," << motor_torque << std::endl;
}

void exportVehicle2DConfig(Vehicle2DConfig &Vehicle2DConfig, std::string filename)
{
     std::ofstream config_file(filename);
     if (!config_file.is_open())
     {
          std::cerr << "Failed to open file: vehicle_config.csv" << std::endl;
          return;
     }

     config_file << "wheelbase_m, track_width_m, steer_max_rad, mass_kg, Izz_kgm2, z_cg_m, a_m, r_wheel_m, I_wheel_kgm2, Tmax_Nm, diff_damping_Ns/m,"
                 << "tire_B, tire_C, tire_D, tire_E, tire_f" << std::endl;
     double B, C, D, E, f;
     Vehicle2DConfig.getTireConfig().getTireParams(B, C, D, E, f);

     config_file << Vehicle2DConfig.getWheelbase() << ","
                 << Vehicle2DConfig.getTrackWidth() << ","
                 << Vehicle2DConfig.getSteerMax() << ","
                 << Vehicle2DConfig.getMass() << ","
                 << Vehicle2DConfig.getIzz() << ","
                 << Vehicle2DConfig.getZcg() << ","
                 << Vehicle2DConfig.getA() << ","
                 << Vehicle2DConfig.getWheelRadius() << ","
                 << Vehicle2DConfig.getI_wheel() << ","
                 << Vehicle2DConfig.getTmax() << ","
                 << Vehicle2DConfig.getDiffDamping() << ","
                 << B << ","
                 << C << ","
                 << D << ","
                 << E << ","
                 << f << std::endl;

     config_file.close();
}
