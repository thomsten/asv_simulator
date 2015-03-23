#ifndef ASV_SIMULATOR_H
#define ASV_SIMULATOR_H

#include <Eigen/Dense>

/**
 * A General Class for simulating 3DOF surface vessels.
 */
class Vessel
{
 public:
  Vessel();
  ~Vessel();

  /** Initializes the system and its parameters.
   *
   * The parameters are loaded from a yaml-config file specified in a launch
   * file. It defaults to the parameters used in Loe, 2008 (the Viknes 830).
   *
   * @param nh ROS (private) node handle used to fetch parameters from the
   * parameter server
   */
  void initialize(ros::NodeHandle nh);

  /** Performs the numerical integration to update the system.
   * 
   * NOTE: The system implements two different controllers:
   * speed and heading control or speed yaw rate control. If the heading set
   * point is specified as inf (psi_d == inf), yaw rate is controlled, else the
   * heading controller is used.
   *  
   * @param u_d The desired speed (control input)
   * @param psi_d The desired heading (control input)
   * @param r_d The desired yaw rate (control input)
   */
  void updateSystem(double u_d, double psi_d, double r_d);

  /**
   * @todo Is this used? Remove.
   */
  void printPose();

  /** Set new values to the state vector [eta, nu]
   *
   * @param eta Position and orientation (x, y, psi)
   * @param nu Velocity (u, v, r)
   */
  void setState(Eigen::Vector3d eta, Eigen::Vector3d nu);

  /** Copies the state data [eta, nu] into the given vectors. Note the
   * call-by-reference.
   * 
   * @param eta Position and orientation (x, y, psi)
   * @param nu Velocity (u, v, r)
   */
  void getState(Eigen::Vector3d &eta, Eigen::Vector3d &nu);

  /** Get the update interval (used to determine ROS loop rate)
   */
  double getDT();

 private:
  /** Determines the generalized force vector based on the control input.
   *
   * Implements two controllers:
   * 1. Speed and heading control
   * 2. Speed and yaw rate control
   *
   * If psi_d == inf, the latter is selected. The speed controller and yaw rate
   * controllers are P-controllers with feedback linearization. The heading
   * controller is a conventional PD-controller.
   *
   * @param u_d The desired speed (control input)
   * @param psi_d The desired heading (control input)
   * @param r_d The desired yaw rate (control input)
   */
  void updateControlInput(double u_d, double psi_d, double r_d);
  
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
