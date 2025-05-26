#pragma once

#include "Vehicle2DData.hpp"
#include "TireConfig.hpp"
#include <string>

enum DrivetrainType_E
{
    RWD = 0,
    AWD = 1,
    FWD = 2
};

class Vehicle2DConfig // vehicle parameter configuration
{
public:
    Vehicle2DConfig();
    Vehicle2DConfig(double wheelbase_, double track_width_, double steer_max_,
                    double mass_, double Izz_,
                    double z_cg_, double a_, double r_wheel_, double I_wheel_,
                    double Tmax_, double Tzero_, double Tnegmax_, double Pmax_, double Pnegmax_, double gear_ratio_,
                    double brake_Tmax_, double brake_bias_,
                    DrivetrainType_E drivetrain_type_, double diff_damping_, TireConfig tire_config_,
                    double CDx_, double CMz_, double Af_, double rho_);

    static Vehicle2DConfig loadFromFile(const std::string &filename);

    double getMass() const;
    double getIzz() const;
    double getWheelbase() const;
    double getTrackWidth() const;
    double getSteerMax() const;
    double getZcg() const;
    double getA() const;
    double getWheelRadius() const;
    double getTmax() const;
    double getTzero() const;
    double getTnegmax() const;
    double getPmax() const;
    double getPnegmax() const;
    double getBrakeTmax() const;
    double getBrakeBias() const;
    DrivetrainType_E getDrivetrainType() const;
    double getDiffDamping() const;
    double getI_wheel() const;
    double getGearRatio() const;
    double getCDx() const;
    double getCMz() const;
    double getFrontalArea() const;
    double getRho() const;
    TireConfig &getTireConfig();

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
    double Tzero;
    double Tnegmax;
    double Pmax;
    double Pnegmax;
    double gear_ratio;
    double brake_Tmax;
    double brake_bias;
    DrivetrainType_E drivetrain_type;
    double diff_damping;
    TireConfig tire_config;
    double CDx;
    double CMz;
    double Af;
    double rho;

    friend class Vehicle2D;
};
