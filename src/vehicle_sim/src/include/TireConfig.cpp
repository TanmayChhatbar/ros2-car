#include "TireConfig.hpp"
#include <cmath> // for std::abs, std::min, std::sqrt

TireConfig::TireConfig()
    : B(0.7), C(1.5), D(1.2), E(0.8), f(100.0), coeff_rr(0.03), coeff_stiction(.008) {}
TireConfig::TireConfig(double B_, double C_, double D_, double E_, double f_, double coeff_rr_, double coeff_stiction_)
    : B(B_), C(C_), D(D_), E(E_), f(f_), coeff_rr(coeff_rr_), coeff_stiction(coeff_stiction_) {}

// tire forces
void TireConfig::setTireParams(const double B_, const double C_, const double D_, const double E_, const double f_)
{
    B = B_;
    C = C_;
    D = D_;
    E = E_;
    f = f_;
}
void TireConfig::getTireParams(double &B_, double &C_, double &D_, double &E_, double &f_) const
{
    B_ = B;
    C_ = C;
    D_ = D;
    E_ = E;
    f_ = f;
}

void TireConfig::setCoeffRR(const double coeff_rr_)
{
    coeff_rr = coeff_rr_;
}
double TireConfig::getCoeffRR() const
{
    return coeff_rr;
}

void TireConfig::calcTireForces(const double slip_angle, const double slip_ratio, const double w_wheel, const double Fz_wheel, double &Fx_wheel, double &Fy_wheel)
{
    // slip_angle [deg]
    // slip_ratio [ratio]
    double slip_angle_corr = slip_angle * std::min(1.0, std::abs(w_wheel) * 0.01);
    double slipNet = std::sqrt(slip_angle_corr * slip_angle_corr + slip_ratio * slip_ratio * f * f);
    if (slipNet < 1e-9)
    {
        Fx_wheel = 0.0;
        Fy_wheel = 0.0;
        return;
    }

    // pacejka tire formula
    const double Fnet = D * std::sin(C * std::atan(B * slipNet - E * (B * slipNet - std::atan(B * slipNet))));

    // calculate forces in each direction
    const double F_rolling_resistance = coeff_rr * Fz_wheel;
    const double F_rolling_stiction = coeff_stiction * Fz_wheel * std::copysign(std::min(std::abs(w_wheel)*100, 1.0), w_wheel);
    const double F_partial = Fz_wheel * Fnet / slipNet;
    Fx_wheel = F_partial * (slip_ratio * f) - F_rolling_resistance - F_rolling_stiction;
    Fy_wheel = -F_partial * slip_angle_corr; // * std::min(1.0, std::abs(w_wheel)*10+0.01);
}
