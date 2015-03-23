#ifndef ASV_SIMULATOR_H
#define ASV_SIMULATOR_H

#include <Eigen/Dense>

// Constants
/// @todo Use rosparam to get paramaters and make the model more general.
// const double DT  = 0.05;

// const double M   = 3300.0;
// const double I_Z = 1320.0;

// const double d1u = 16.6;
// const double d1v = 9900.0;
// const double d1r = 330.0;
// const double d2u = 8.25;
// const double d2v = 330.0;
// const double d2r = 0.0;

// const double L_RUDR = 4.0;
// const double FX_MAX = 2310.0;
// const double FY_MAX = 28.8;
// const double KP_U   = 0.1;
// const double KP_PSI = 5.0;
// const double KD_PSI = 1.0;
// const double KP_R   = 8.0;


/**
 * A General Class for simulating 3DOF surface vessels.
 */
class Vessel
{
 public:
  Vessel();
  ~Vessel();

  /// Set parameters of the system and its initial state.
  void initialize(ros::NodeHandle nh);
  void updateSystem(double u_d, double psi_d, double r_d);
  void printPose();
  void setState(Eigen::Vector3d eta, Eigen::Vector3d nu);
  void getState(Eigen::Vector3d &eta, Eigen::Vector3d &nu);
  double getDT();

 private:
  void updateControlInput(double, double, double);
  
  Eigen::Vector3d eta;
  Eigen::Vector3d nu; 
  Eigen::Vector3d tau;

  Eigen::Matrix3d Minv;
  Eigen::Vector3d Cvv;
  Eigen::Vector3d Dvv;

  double DT;

  // Mass and inertia
  double M;
  double I_z;
  
  // Added mass parameters
  double X_udot, Y_vdot, Y_rdot, N_vdot, N_rdot;
  
  // Damping parameters
  double X_u, Y_v, Y_r, N_v, N_r;
  double X_uu, Y_vv, N_rr, X_uuu, Y_vvv, N_rrr;

  // Other
  double Fx_min, Fx_max, Fy_min, Fy_max;
  double rudder_length;

  double Kp_u, Kp_psi, Kd_psi, Kp_r;

};



#endif
