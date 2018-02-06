#include "mav_trim/MavTrim.h"

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);

  double x = 0.5;
  const double initial_x = x;

  ceres::Problem problem;

  ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);

  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  
  return 0;
}

namespace mav_trim
{
  MavTrim::MavTrim() :
    nh_(ros::NodeHandle()),
    params_(nh_)
  {
    ros::NodeHandle nh_private("~");

    // Create inertia matrix
    J << params_.Jx, 0, -params_.Jxz,
         0,     params_.Jy,        0,
         -params_.Jxz, 0, params_.Jz;

    J_inv = J.inverse();
  }

  Vector12f MavTrim::dynamics(Vector12f state, Vector6f wrench)
  {
    Vector12f state_dot = Eigen::MatrixXf::Zero(12,1);

    //state derivatives
    Eigen::Vector3f dpos = Eigen::Vector3f::Zero();
    Eigen::Vector3f datt = Eigen::Vector3f::Zero();
    Eigen::Vector3f dvel = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega = Eigen::Vector3f::Zero();

    //unpack state
    Eigen::Vector3f pos;
    Eigen::Vector3f att;
    Eigen::Vector3f vel;
    Eigen::Vector3f omega;
    Eigen::Vector3f force;
    Eigen::Vector3f torque;

    pos << state(0), state(1), state(2);
    att << state(3), state(4), state(5);
    vel << state(6), state(7), state(8);
    omega << state(9), state(10), state(11);

    force << wrench(0), wrench(1), wrench(2); 
    torque << wrench(3), wrench(4), wrench(5); 

    float cphi = std::cos(att(0));
    float sphi = std::sin(att(0));
    float ctheta = std::cos(att(1));
    float stheta = std::sin(att(1));
    float ttheta = std::tan(att(1));
    float cpsi = std::cos(att(2));
    float spsi = std::sin(att(2));


    // Transformation to convert body into vehicle (i.e. NED)
    // note: vehicle to body is given, then we take the inverse, but Eigen uses active rotations
    // instead of passive rotations so we take the inverse again
    Eigen::Quaternion<float> R_vb (Eigen::AngleAxisf(att(2), Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(att(1), Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(att(0), Eigen::Vector3f::UnitX())); 

    // I used this matrix to check my sanity with the quaternion
    //Eigen::Matrix3f R_vb;
    //R_vb << ctheta*cpsi, sphi*stheta*spsi - cphi*spsi, cphi*stheta*cpsi + sphi*spsi,
    //     ctheta*spsi, sphi*stheta*spsi + cphi*cpsi, cphi*stheta*spsi - sphi*cpsi,
    //     -stheta, sphi*ctheta, cphi*ctheta;

    // Make matrix for datt
    Eigen::Matrix3f AttD;

    AttD << 1.0, sphi*ttheta, cphi*ttheta,
            0.0, cphi, -sphi,
            0.0, sphi/ctheta, cphi/ctheta;

    // Calculate derivatives
    dpos = R_vb*vel;
    dvel = -omega.cross(vel) + force/params_.m;
    datt = AttD*omega;
    domega = J_inv*(-omega.cross(J*omega)+torque);

    state_dot << dpos, datt, dvel, domega;

    return state_dot;
  }

  Vector6f MavTrim::calcWrench(Eigen::Vector3f trim, Eigen::Vector3f abr)
  {
    float gamma = trim(2);
    float phi = abr(2);
    float theta = trim(2) + abr(0);

    float alpha = abr(0);
    float beta = abr(1);

    float V_a = trim(0);
    float V_a2 = std::pow(V_a,2);

    float R = trim(1);

    Eigen::Vector3f linear;
    Eigen::Vector3f angular;

    linear << V_a*std::cos(alpha)*std::cos(beta),
              V_a*std::sin(beta),
              V_a*std::sin(alpha)*std::cos(beta);

    angular << -V_a*std::sin(theta)*std::cos(gamma)/R,
               V_a*std::sin(phi)*std::cos(theta)*std::cos(gamma)/R,
               V_a*std::cos(phi)*std::cos(theta)*std::cos(gamma)/R;

    Command command;
    command.dele = ((params_.Jxz*(std::pow(angular(0),2) - std::pow(angular(2),2))+
          (params_.Jx - params_.Jz)*angular(0)*angular(2))
        /(.5*params_.rho*std::pow(V_a,2)*params_.c*params_.S) 
        - params_.C_m0 - params_.C_malpha*alpha 
        - params_.C_mq*(0.5*params_.c*angular(1)/V_a))/params_.C_mdele;

    // calculate gravity
    Eigen::Vector3f Force_g = Eigen::Vector3f::Zero();
    Force_g <<  0.0, 0.0, params_.m*params_.g;

    // We do the same thing as in mav_dynamics and take the inverse because
    // active vs passive rotations
    Eigen::Quaternion<float> R_bv1 (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()) *
              Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX());
    R_bv1 = R_bv1.inverse();

    /*
     * Gravitational forces
     */

    // Rotate gravity into body frame
    Force_g << 0.0, 0.0, params_.m*params_.g;
    Force_g = R_bv1 *Force_g;

    /*
     * Aerodynamic forces and moments
     */

    Eigen::Vector3f Force_aero = Eigen::Vector3f::Zero();
    Eigen::Vector3f Moment_aero = Eigen::Vector3f::Zero();

    // Calculate angle of attack and rotation from stability to body frame
    // Note: we use the inverse to convert from active rotations to passive rotations
    Eigen::Quaternion<float> R_bs(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitY()));
    R_bs = R_bs.inverse();

    // Calculate longitudinal forces and moments
    float AR = std::pow(params_.b,2)/params_.S;
    float sigma_alpha = sigma(alpha);
    float C_Lflat = 2*sgn(alpha)*std::pow(std::sin(alpha),2)*std::cos(alpha);
    float C_Lalpha = (1.0 - sigma_alpha)*(params_.C_L0 + params_.C_Lalpha*alpha) + sigma_alpha*C_Lflat;
    //float C_Lalpha = params_.C_L0 + params_.C_Lalpha*alpha;
    float C_Dalpha = params_.C_Dp + std::pow(params_.C_L0 + params_.C_Lalpha*alpha, 2.0) /
      (M_PI*params_.e*AR);
    // float C_Dalpha = params_.C_D0 + params_.C_Dalpha*alpha;

    float F_lift = .5*params_.rho*V_a2*params_.S*(C_Lalpha + 
        params_.C_Lq*(.5*params_.c/V_a)*0*angular(1) + params_.C_Ldele*command.dele); 
    float F_drag = .5*params_.rho*V_a2*params_.S*(C_Dalpha + 
        params_.C_Dq*(.5*params_.c/V_a)*0*angular(1) + params_.C_Ddele*command.dele); 

    Force_aero << -F_drag, 0, -F_lift;
    // coded rotation matrix to check quaternion math
    //Eigen::Matrix3f m;
    //m << std::cos(alpha), 0, -std::sin(alpha),
    //     0, 0, 0,
    //     std::sin(alpha), 0, std::cos(alpha);

    Force_aero = R_bs*Force_aero;

    Moment_aero(1) = .5*params_.rho*V_a2*params_.S*params_.c*(params_.C_m0 +
        params_.C_malpha*alpha + params_.C_mq*(.5*params_.c/V_a)*angular(1) + 
        params_.C_mdele*command.dele);

  // Calculate lateral forces and moments
    Force_aero(1) = .5*params_.rho*V_a2*params_.S*(params_.C_Y0 +
       params_.C_Ybeta*beta + params_.C_Yp*(.5*params_.b/V_a)*angular(0) + 
       params_.C_Yr*(.5*params_.b/V_a)*angular(2) + params_.C_Ydela*command.dela +
       params_.C_Ydelr*command.delr);

    Moment_aero(0) = .5*params_.rho*V_a2*params_.S*params_.b*(params_.C_l0 +
       params_.C_lbeta*beta + params_.C_lp*(.5*params_.b/V_a)*angular(0) + 
       params_.C_lr*(.5*params_.b/V_a)*angular(2) + params_.C_ldela*command.dela +
       params_.C_ldelr*command.delr);

    Moment_aero(2) = .5*params_.rho*V_a2*params_.S*params_.b*(params_.C_n0 +
       params_.C_nbeta*beta + params_.C_np*(.5*params_.b/V_a)*angular(0) + 
       params_.C_nr*(.5*params_.b/V_a)*angular(2) + params_.C_ndela*command.dela +
       params_.C_ndelr*command.delr);

    /*
     * Propulsion Forces and Moments
     */
    Eigen::Vector3f Force_prop = Eigen::Vector3f::Zero();
    Eigen::Vector3f Moment_prop = Eigen::Vector3f::Zero();

    Force_prop(0) = .5*params_.rho*params_.Sprop*params_.C_prop*
      (std::pow(params_.k_motor*command.delt, 2) - V_a);

    Moment_prop(0) = -params_.k_Tp*std::pow(params_.k_Omega*command.delt,2);

    /*
     * Sum of Forces and Moments
     */
    Eigen::Vector3f force = Eigen::Vector3f::Zero();
    Eigen::Vector3f torque = Eigen::Vector3f::Zero();
    force = Force_prop + Force_g + Force_aero;
    torque = Moment_prop + Moment_aero;
    Vector6f wrench;
    wrench << force, torque;

    return wrench;
  }

  float MavTrim::sigma(float alpha)
  {
    return (1.0 + std::exp(-params_.M*(alpha - params_.alpha0)) 
        + std::exp(params_.M*(alpha+params_.alpha0))) /
      ((1.0 + std::exp(-params_.M*(alpha - params_.alpha0)))*
       (1.0 + std::exp(params_.M*(alpha+params_.alpha0))));
  }

}
