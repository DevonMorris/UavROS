#include <ros/ros.h>

namespace mav_params
{

class MavParams
{
  public:
    // constructor
    MavParams(ros::NodeHandle& nh_);

    double g; // acceleration due to gravity
    double m; // mass

    // moments of interia
    double Jx; 
    double Jy;
    double Jz;
    double Jxz;

    double S;
    double b;
    double c;
    double Sprop;
    double rho;
    double k_motor;
    double k_Tp;
    double k_Omega;
    double e;

    // Longitudinal Coeffs
    double C_L0;
    double C_D0;
    double C_m0;
    double C_Lalpha;
    double C_Dalpha;
    double C_malpha;
    double C_Lq;
    double C_Dq;
    double C_mq;
    double C_Ldele;
    double C_Ddele;
    double C_mdele;
    double C_prop;
    double M;
    double alpha0;
    double eps;
    double C_Dp;

    // Lateral Coeffs
    double C_Y0;
    double C_l0;
    double C_n0;
    double C_Ybeta;
    double C_lbeta;
    double C_nbeta;
    double C_Yp;
    double C_lp;
    double C_np;
    double C_Yr;
    double C_lr;
    double C_nr;
    double C_Ydela;
    double C_ldela;
    double C_ndela;
    double C_Ydelr;
    double C_ldelr;
    double C_ndelr;

    // Trim Conditions
    double Va;
    double R;
    double gamma;
};

} 
