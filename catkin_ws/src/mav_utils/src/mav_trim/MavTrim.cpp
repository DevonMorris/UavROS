#include "mav_trim/MavTrim.h"

namespace mav_trim
{
  MavTrim::MavTrim() :
    nh_(ros::NodeHandle())
  {
    ros::NodeHandle nh_private("~");
  }

  template<typename T>
  Vector12f TrimFunctor::dynamics(T abr)
  {
    Vector12f state_dot = Eigen::MatrixXf::Zero(12,1);

    // Gamma terms
    // not to be confused with gamma
    float Gamma = p_.Jx*p_.Jz - std::pow(p_.Jxz,2);
    float Gamma1 = p_.Jxz*(p_.Jx - p_.Jy + p_.Jz)/Gamma;
    float Gamma2 = (p_.Jz*(p_.Jz - p_.Jy) + std::pow(p_.Jxz,2))/Gamma;
    float Gamma3 = p_.Jz/Gamma;
    float Gamma4 = p_.Jxz/Gamma;
    float Gamma5 = (p_.Jz - p_.Jx)/p_.Jy;
    float Gamma6 = p_.Jxz/p_.Jy;
    float Gamma7 = ((p_.Jx - p_.Jy)*p_.Jx + std::pow(p_.Jxz,2))/Gamma;
    float Gamma8 = p_.Jx/Gamma;

    // Roll rate and yaw rate coefficients
    float C_p0 = Gamma3*p_.C_l0 + Gamma4*p_.C_n0;
    float C_pbeta = Gamma3*p_.C_lbeta + Gamma4*p_.C_nbeta;
    float C_pp = Gamma3*p_.C_lp + Gamma4*p_.C_np;
    float C_pr = Gamma3*p_.C_lr + Gamma4*p_.C_nr;
    float C_pdela = Gamma3*p_.C_ldela + Gamma4*p_.C_ndela;
    float C_pdelr = Gamma3*p_.C_ldelr + Gamma4*p_.C_ndelr;
    float C_r0 = Gamma4*p_.C_l0 + Gamma8*p_.C_n0;
    float C_rbeta = Gamma4*p_.C_lbeta + Gamma8*p_.C_nbeta;
    float C_rp = Gamma4*p_.C_lp + Gamma8*p_.C_np;
    float C_rr = Gamma4*p_.C_lr + Gamma8*p_.C_nr;
    float C_rdela = Gamma4*p_.C_ldela + Gamma8*p_.C_ndela;
    float C_rdelr = Gamma4*p_.C_ldelr + Gamma8*p_.C_ndelr;

    // calculate trim state of the system
    float phi = abr[2];
    float theta = abr[2] + trim(2);
    float u = trim(0)*std::cos(abr[0])*std::cos(abr[1]);
    float v = trim(0)*std::sin(abr[0]);
    float w = trim(0)*std::sin(abr[0])*std::cos(abr[1]);
    float p = -trim(0)*std::sin(abr[0] + trim(2))/trim(1);
    float q = trim(0)*std::sin(abr[2])*std::cos(abr[0] + trim(2))/trim(1);
    float r = trim(0)*std::cos(abr[2])*std::cos(abr[0] + trim(2))/trim(1);

    float cphi = std::cos(phi);
    float sphi = std::sin(phi);
    float ctheta = std::cos(theta);
    float stheta = std::sin(theta);
    float ttheta = std::tan(theta);

    // calculate aerodynamics
    float V_a2 = std::pow(trim(0),2);
    float AR = std::pow(p_.b,2)/p_.S;
    float sigma_alpha = sigma(abr[0]);
    float C_Lflat = 2*sgn(abr[0])*std::pow(std::sin(abr[0]),2)*std::cos(abr[0]);
    float C_Lalpha = (1.0 - sigma_alpha)*(p_.C_L0 + p_.C_Lalpha*abr[0]) + sigma_alpha*C_Lflat;
    float C_Dalpha = p_.C_Dp + std::pow(p_.C_L0 + p_.C_Lalpha*abr[0], 2.0) /
      (M_PI*p_.e*AR);
    float C_X = -C_Dalpha*std::cos(abr[0]) + C_Lalpha*std::sin(abr[0]); 
    float C_Xq = -p_.C_Dq*std::cos(abr[0]) + p_.C_Lq*std::sin(abr[0]);
    float C_Xdele = -p_.C_Ddele*std::cos(abr[0]) + p_.C_Ldele*std::sin(abr[0]);
    float C_Z = -C_Dalpha*std::sin(abr[0]) - C_Lalpha*std::cos(abr[0]);
    float C_Zq = -p_.C_Dq*std::sin(abr[0]) - p_.C_Lq*std::cos(abr[0]);
    float C_Zdele = -p_.C_Ddele*std::sin(abr[0]) - p_.C_Ldele*std::cos(abr[0]);

    // calculate trim inputs
    Command command;
    command.dele = ((p_.Jxz*(std::pow(p,2) - std::pow(r,2))+
          (p_.Jx - p_.Jz)*p*r)
        /(.5*p_.rho*V_a2*p_.c*p_.S) 
        - p_.C_m0 - p_.C_malpha*abr[0] 
        - p_.C_mq*(0.5*p_.c*q/trim(0)))/p_.C_mdele;

    command.delt = std::sqrt(2.0*p_.m*(-r*v + q*w +
        p_.g*std::sin(theta) - p_.rho*V_a2*p_.S*
        (C_X+C_Xq*(.5*p_.c*q/trim(0))+C_Xdele*command.dele)) /
        (p_.rho*p_.Sprop*p_.C_prop*std::pow(p_.k_motor,2)) + 
        V_a2/(std::pow(p_.k_motor, 2)));

    Eigen::Matrix2f mat;
    mat << C_pdela, C_pdelr,
           C_rdela, C_rdelr;
    mat = mat.inverse().eval();
    Eigen::Vector2f vec;
    vec(0) = (-Gamma1*p*q + Gamma2*q*r)/(0.5*p_.rho*V_a2*p_.S*p_.b) -
      C_p0 - C_pbeta*abr[1] - 0.5*C_pp*p_.b*p/trim(0) - 0.5*C_pr*p_.b*r/trim(0);
    vec(1) = (-Gamma7*p*q + Gamma2*q*r)/(0.5*p_.rho*V_a2*p_.S*p_.b) -
      C_r0 - C_rbeta*abr[1] - 0.5*C_rp*p_.b*p/trim(0) - 0.5*C_rr*p_.b*r/trim(0);
    vec = mat*vec.eval();

    command.dela = vec(0);
    command.delr = vec(1);

    state_dot(0) = 0.0;
    state_dot(1) = 0.0;
    state_dot(2) = u*stheta - v*sphi*ctheta - w*cphi*ctheta;
    state_dot(3) = r*v-q*w - p_.g*stheta + 
      0.5*p_.rho*V_a2*p_.S*(C_X + 0.5*C_Xq*(p_.c*q/trim(0)) + C_Xdele*command.dele)/p_.m +
      0.5*p_.Sprop*p_.C_prop;
    state_dot(4) = p*w - r*u + p_.g*ctheta*sphi + 
          .5*p_.rho*V_a2*p_.S*(p_.C_Y0 + p_.C_Ybeta*abr[1] + 
            0.5*p_.C_Yp*p_.b*p/trim(0) + 0.5*p_.C_Yr*p_.b*r/trim(0) + 
            p_.C_Ydela*command.dela + p_.C_Ydelr*command.delr)/p_.m;
    state_dot(5) = q*u - p*v + p_.g*ctheta*cphi +
      0.5*p_.rho*V_a2*p_.S*(C_Z + 0.5*C_Zq*p_.c*q/trim(0) + C_Zdele*command.dele)/p_.m;
    state_dot(6) = p + q*sphi*ttheta + r*cphi*ttheta;
    state_dot(7) = q*cphi - r*sphi;
    state_dot(8) = q*sphi/ctheta + r*cphi/ctheta;
    state_dot(9) = Gamma1*p*q - Gamma2*q*r + .5*p_.rho*V_a2*p_.S*p_.b*
      (C_p0 + C_pbeta*abr[1] + 0.5*C_pp*p_.b*p/trim(0) + 0.5*C_pr*p_.b*r/trim(0) + 
       C_pdela*command.dela + C_pdelr*command.delr);
    state_dot(10) = Gamma5*p*r - Gamma6*(std::pow(p,2) - std::pow(r,2)) +
      0.5*p_.rho*V_a2*p_.S*p_.c*(p_.C_m0 + p_.C_malpha*abr[0] + 
          0.5*p_.C_mq*p_.c*q/trim(0) + p_.C_mdele*command.dele)/p_.Jy;
    state_dot(11) = Gamma7*p*q - Gamma1*q*r + 0.5*p_.rho*V_a2*p_.S*p_.b*
      (C_r0 + C_rbeta*abr[1] + 0.5*C_rp*p_.b*p/trim(0) +
       0.5*C_rr*p_.b*r/trim(0) + C_rdela*command.dela + C_rdelr*command.delr);

    return state_dot;
  }

  bool MavTrim::trim_cb_(mav_utils::Trim::Request req, mav_utils::Trim::Response resp)
  {
    trim << req.trims.Va, req.trims.gamma, req.trims.R;
    
    ceres::Problem problem;

    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<TrimFunctor, 12, 3>(new TrimFunctor(this));

    problem.AddResidualBlock(cost_function, nullptr, &x);

    // Run the solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    
    return true;
  }

  float TrimFunctor::sigma(float alpha)
  {
    return (1.0 + std::exp(-p_.M*(alpha - p_.alpha0)) 
        + std::exp(p_.M*(alpha+p_.alpha0))) /
      ((1.0 + std::exp(-p_.M*(alpha - p_.alpha0)))*
       (1.0 + std::exp(p_.M*(alpha+p_.alpha0))));
  }

  TrimFunctor::TrimFunctor(MavTrim* mtrim):
    p_(mtrim->nh_)
  {
    trim = mtrim->trim;
  }

}
