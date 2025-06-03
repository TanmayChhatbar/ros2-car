#pragma once

class TireConfig // tire config and forces
{
public:
    TireConfig();
    TireConfig(double B_, double C_, double D_, double E_, double f_, double coeff_rr_, double coeff_stiction_);

    void setTireParams(const double B_, const double C_, const double D_, const double E_, const double f_);
    void getTireParams(double &B_, double &C_, double &D_, double &E_, double &f_) const;

    void setCoeffRR(const double coeff_rr_);
    double getCoeffRR() const;

    void calcTireForces(const double slip_angle, const double slip_ratio, const double w_wheel, const double Fz_wheel, double &Fx_wheel, double &Fy_wheel);

private:
    double B;              // stiffness factor
    double C;              // shape factor
    double D;              // peak factor
    double E;              // curvature factor
    double f;              // coeff of relative stiffness for long and lateral directions
    double coeff_rr;       // rolling resistance coefficient
    double coeff_stiction; // static friction coefficient
    friend class Vehicle2DConfig;
    friend class Vehicle2D;
};
