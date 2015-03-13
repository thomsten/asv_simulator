#ifndef ASV_SIMULATOR_H
#define ASV_SIMULATOR_H

#include <Eigen/Dense>

// Constants
/// @todo Use rosparam to get paramaters and make the model more general.
const double DT  = 0.05;

const double M   = 3300.0;
const double I_Z = 1320.0;

const double d1u = 16.6;
const double d1v = 9900.0;
const double d1r = 330.0;
const double d2u = 8.25;
const double d2v = 330.0;
const double d2r = 0.0;

const double L_RUDR = 4.0;
const double FX_MAX = 2310.0;
const double FY_MAX = 28.8;
const double KP_U   = 0.1;
const double KP_PSI = 5.0;
const double KD_PSI = 1.0;
const double KP_R   = 8.0;


/**
 * A General Class for simulating 3DOF surface vessels.
 */
class Vessel
{
 public:
  Vessel();
  void updateSystem(double, double, double);
  void printPose();
  void setState(Eigen::Vector3d eta, Eigen::Vector3d nu);
  void getState(Eigen::Vector3d &eta, Eigen::Vector3d &nu);
  double getDT();
  ~Vessel();

 private:
  void updateControlInput(double, double, double);
  
  Eigen::Vector3d eta;
  Eigen::Vector3d nu; 
  Eigen::Vector3d tau;

  Eigen::Matrix3d Minv;
  Eigen::Vector3d Cvv;
  Eigen::Vector3d Dvv;

};



#endif
