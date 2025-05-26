#include "Vehicle2DConfig.hpp"
#include "TireConfig.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include "json.hpp"

Vehicle2DConfig::Vehicle2DConfig()
    : wheelbase(2.57), track_width(1.79), steer_max(30.0 * M_PI / 180.0),
      mass(1250.0), Izz(1800.0), z_cg(0.6), a(1.24), r_wheel(0.3125), I_wheel(2.0),
      Tmax(205), Tnegmax(250.0), Pmax(152e3), Pnegmax(100e3), gear_ratio(3.626 * 4.1),
      brake_Tmax(2e4), brake_bias(0.55),
      drivetrain_type(RWD), diff_damping(10.0), tire_config(),
      CDx(0.3), CMz(0.0), Af(1.775 * (1.425 - 0.1)), rho(1.225) {} // Toyota 86 (https://en.wikipedia.org/wiki/Toyota_86)
Vehicle2DConfig::Vehicle2DConfig(double wheelbase_, double track_width_, double steer_max_,
                                 double mass_, double Izz_, double z_cg_, double a_, double r_wheel_, double I_wheel_,
                                 double Tmax_, double Tzero_, double Tnegmax_, double Pmax_, double Pnegmax_, double gear_ratio_,
                                 double brake_Tmax_, double brake_bias_,
                                 DrivetrainType_E drivetrain_type_, double diff_damping_, TireConfig tire_config_,
                                 double CDx_, double CMz_, double Af_, double rho_)
    : wheelbase(wheelbase_), track_width(track_width_), steer_max(steer_max_),
      mass(mass_), Izz(Izz_), z_cg(z_cg_), a(a_), r_wheel(r_wheel_), I_wheel(I_wheel_),
      Tmax(Tmax_), Tzero(Tzero_), Tnegmax(Tnegmax_), Pmax(Pmax_), Pnegmax(Pnegmax_), gear_ratio(gear_ratio_),
      brake_Tmax(brake_Tmax_), brake_bias(brake_bias_),
      drivetrain_type(drivetrain_type_), diff_damping(diff_damping_), tire_config(tire_config_),
      CDx(CDx_), CMz(CMz_), Af(Af_), rho(rho_) {}

double Vehicle2DConfig::getMass() const { return mass; }
double Vehicle2DConfig::getIzz() const { return Izz; }
double Vehicle2DConfig::getWheelbase() const { return wheelbase; }
double Vehicle2DConfig::getTrackWidth() const { return track_width; }
double Vehicle2DConfig::getSteerMax() const { return steer_max; }
double Vehicle2DConfig::getZcg() const { return z_cg; }
double Vehicle2DConfig::getA() const { return a; }
double Vehicle2DConfig::getWheelRadius() const { return r_wheel; }
double Vehicle2DConfig::getTmax() const { return Tmax; }
double Vehicle2DConfig::getTnegmax() const { return Tnegmax; }
double Vehicle2DConfig::getTzero() const { return Tzero; }
double Vehicle2DConfig::getPmax() const { return Pmax; }
double Vehicle2DConfig::getPnegmax() const { return Pnegmax; }
double Vehicle2DConfig::getBrakeTmax() const { return brake_Tmax; }
double Vehicle2DConfig::getBrakeBias() const { return brake_bias; }
DrivetrainType_E Vehicle2DConfig::getDrivetrainType() const { return drivetrain_type; }
double Vehicle2DConfig::getDiffDamping() const { return diff_damping; }
double Vehicle2DConfig::getI_wheel(void) const { return I_wheel; }
double Vehicle2DConfig::getGearRatio(void) const { return gear_ratio; }
double Vehicle2DConfig::getCDx(void) const { return CDx; }
double Vehicle2DConfig::getCMz(void) const { return CMz; }
double Vehicle2DConfig::getFrontalArea(void) const { return Af; }
double Vehicle2DConfig::getRho(void) const { return rho; }
TireConfig &Vehicle2DConfig::getTireConfig() { return tire_config; }

Vehicle2DConfig Vehicle2DConfig::loadFromFile(const std::string& filename) {
  try {
      // read JSON file
      std::ifstream file(filename);
      if (!file.is_open()) {
          std::cerr << "Error: Could not open file " << filename << std::endl;
          return Vehicle2DConfig(); // return default
      }
      
      nlohmann::json j;
      file >> j;
      
      const auto& vehicle = j;
      
      TireConfig tire_config;
      if (vehicle.contains("tire_config")) {
          const auto& tc = vehicle["tire_config"];
          tire_config = TireConfig(
              tc.value("B", 0.7),
              tc.value("C", 1.5),
              tc.value("D", 1.0),
              tc.value("E", 0.8),
              tc.value("f", 100.0),
              tc.value("coeff_rr", 0.03),
              tc.value("coeff_stiction", 0.008)
          );
      }
      
      // Create and return the Vehicle2DConfig
      return Vehicle2DConfig(
          vehicle.value("wheelbase", 2.57),
          vehicle.value("track_width", 1.79),
          vehicle.value("steer_max", 30.0 * M_PI / 180.0),
          vehicle.value("mass", 1250.0),
          vehicle.value("Izz", 1800.0),
          vehicle.value("z_cg", 0.6),
          vehicle.value("a", 1.24),
          vehicle.value("r_wheel", 0.3125),
          vehicle.value("I_wheel", 2.0),
          vehicle.value("Tmax", 205.0),
          vehicle.value("Tzero", 30.0),
          vehicle.value("Tnegmax", 250.0),
          vehicle.value("Pmax", 152e3),
          vehicle.value("Pnegmax", 100e3),
          vehicle.value("gear_ratio", 3.626 * 4.1),
          vehicle.value("brake_Tmax", 2e4),
          vehicle.value("brake_bias", 0.55),
          vehicle.value("drivetrain_type", RWD),
          vehicle.value("diff_damping", 10.0),
          tire_config,
          vehicle.value("CDx", 0.3),
          vehicle.value("CMz", 0.0),
          vehicle.value("Af", 1.775 * (1.425 - 0.1)),
          vehicle.value("rho", 1.225)
      );
  }
  catch (const std::exception& e) {
      std::cerr << "Error loading vehicle config: " << e.what() << std::endl;
      return Vehicle2DConfig(); // Return default config on error
  }
}
