#include "mav_params/MavParams.h"
#include <string>

namespace mav_params
{

MavParams::MavParams(ros::NodeHandle& nh_)
{
    // grab params and check if they exist
    if (!nh_.getParam("/mav/m", m) ||
        !nh_.getParam("/mav/g", g) ||
        !nh_.getParam("/mav/Jx", Jx) ||
        !nh_.getParam("/mav/Jy", Jy) ||
        !nh_.getParam("/mav/Jz", Jz) ||
        !nh_.getParam("/mav/Jxz", Jxz) ||
        !nh_.getParam("/mav/S", S) ||
        !nh_.getParam("/mav/b", b) ||
        !nh_.getParam("/mav/c", c) ||
        !nh_.getParam("/mav/Sprop", Sprop) ||
        !nh_.getParam("/mav/rho", rho) ||
        !nh_.getParam("/mav/k_motor", k_motor) ||
        !nh_.getParam("/mav/k_Tp", k_Tp) ||
        !nh_.getParam("/mav/k_Omega", k_Omega) ||
        !nh_.getParam("/mav/e", e) ||
        !nh_.getParam("/mav/C_L0", C_L0) ||
        !nh_.getParam("/mav/C_D0", C_D0) ||
        !nh_.getParam("/mav/C_m0", C_m0) ||
        !nh_.getParam("/mav/C_Lalpha", C_Lalpha) ||
        !nh_.getParam("/mav/C_Dalpha", C_Dalpha) ||
        !nh_.getParam("/mav/C_malpha", C_malpha) ||
        !nh_.getParam("/mav/C_Lq", C_Lq) ||
        !nh_.getParam("/mav/C_Dq", C_Dq) ||
        !nh_.getParam("/mav/C_mq", C_mq) ||
        !nh_.getParam("/mav/C_Ldele", C_Ldele) ||
        !nh_.getParam("/mav/C_Ddele", C_Ddele) ||
        !nh_.getParam("/mav/C_mdele", C_mdele) ||
        !nh_.getParam("/mav/C_prop", C_prop) ||
        !nh_.getParam("/mav/M", M) ||
        !nh_.getParam("/mav/alpha0", alpha0) ||
        !nh_.getParam("/mav/eps", eps) ||
        !nh_.getParam("/mav/C_Dp", C_Dp) ||
        !nh_.getParam("/mav/C_Y0", C_Y0) ||
        !nh_.getParam("/mav/C_l0", C_l0) ||
        !nh_.getParam("/mav/C_n0", C_n0) ||
        !nh_.getParam("/mav/C_Ybeta", C_Ybeta) ||
        !nh_.getParam("/mav/C_lbeta", C_lbeta) ||
        !nh_.getParam("/mav/C_nbeta", C_nbeta) ||
        !nh_.getParam("/mav/C_Yp", C_Yp) ||
        !nh_.getParam("/mav/C_lp", C_lp) ||
        !nh_.getParam("/mav/C_np", C_np) ||
        !nh_.getParam("/mav/C_Yr", C_Yr) ||
        !nh_.getParam("/mav/C_lr", C_lr) ||
        !nh_.getParam("/mav/C_nr", C_nr) ||
        !nh_.getParam("/mav/C_Ydela", C_Ydela) ||
        !nh_.getParam("/mav/C_ldela", C_ldela) ||
        !nh_.getParam("/mav/C_ndela", C_ndela)) 
        
    {
      ROS_ERROR("[%s] params not found on rosparam server", ros::this_node::getName().c_str());
      //ros::shutdown();
    }
}

} // end namespace
