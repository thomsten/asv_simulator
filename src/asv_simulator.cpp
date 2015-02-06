#include "ros/ros.h"
#include "asv_simulator.h"

// Standard libraries
#include <cmath>
#include <iostream>

// Linear algebra math
#include <Eigen/Dense>

double normalize_angle(double val);
double normalize_angle(double angle, double angle_ref);


// int main(int argc, char* argv[])
// {

//   ros::init(argc, argv, "asv_simulator_node");
//   ros::start();

//   ROS_INFO("Started ASV Simulator node");


//   Vessel myVessel = Vessel();

//   for (int i=0; i<10; ++i)
//     {
//       myVessel.updateSystem(3.0, 0.0, 0.0);
//       myVessel.printPose();
//     }


//   Eigen::Vector3d v1(0,0,0), v2(4,5,6);

//   v1 = v2;
//   v2(1) = 1000;

//   std::cout << "Eigen test" << std::endl << v1 << std::endl << v2 << std::endl;

//   ros::spin();
//   ros::shutdown();
//   return 0;
// }

/**
 * Constructor
 */
Vessel::Vessel()
{
  Minv <<
    1.0/M, 0.0, 0.0,
    0.0, 1.0/M, 0.0,
    0.0,   0.0, 1.0/I_Z;

  Cvv  = Eigen::Vector3d::Zero();
  Dvv  = Eigen::Vector3d::Zero();

  eta = Eigen::Vector3d::Zero();
  nu  = Eigen::Vector3d::UnitX() * 3.0;
}

/**
 * Destructor
 */
Vessel::~Vessel()
{
}

double Vessel::getDT()
{
  return DT;
}

void Vessel::printPose()
{
  std::cout << eta << std::endl << std::endl;
}

void Vessel::getState(Eigen::Vector3d &eta2, Eigen::Vector3d &nu2)
{
  eta2 = eta;
  nu2 = nu;
}

void Vessel::updateSystem(double u_d, double psi_d, double r_d)
{
  // Ensure -PI <= psi_d < PI
  psi_d = normalize_angle(psi_d);

  Eigen::AngleAxisd rot_z = Eigen::AngleAxisd(eta[2], Eigen::Vector3d::UnitZ());

  // Calculate coriolis and dampening matrices
  Cvv[0] = - M * nu[1] * nu[2];
  Cvv[1] =   M * nu[0] * nu[2];
  //Cvv(2) = 0.0;

  Dvv[0] = d2u*nu[0]*fabs(nu[0]) + d1u*nu[0];
  Dvv[1] = d2v*nu[1]*fabs(nu[1]) + d1v*nu[1];
  Dvv[2] = d1r*nu[2];

  this->updateControlInput(u_d, psi_d, r_d);

  // Integrate system
  eta += DT * (rot_z * nu);
  nu  += DT * (Minv * (tau - Cvv - Dvv));

  // Keep yaw within [-PI,PI)
  eta[2] = normalize_angle(eta[2]);

  //this->printPose();
}


/**
 * Update control inputs.
 *
 * Implements a feedback-linearizing controller.
 */
void Vessel::updateControlInput(double u_d, double psi_d, double r_d)
{
  double Fx = (d1u + d2u*fabs(nu[0]))*nu[0] + nu[1]*nu[2] + KP_U*M*(u_d - nu[0]);
  double Fy = 0.0;

  // If psi_d == inf, then use yaw-rate controller
  if (isinf(psi_d))
    {
      Fy = (d1r + d2r*fabs(nu[2]))*nu[2] + I_Z*KP_R*(r_d - nu[2]);
      Fy *= 1.0 / L_RUDR;
    }
  else
    {
      Fy = (KP_PSI * I_Z / L_RUDR) * ((psi_d - eta[2]) - KD_PSI*nu[2]);
    }

  /// @todo Maybe not realistic with full speed reverse.
  if (fabs(Fx) > FX_MAX)
    {
      Fx = copysign(FX_MAX, Fx);
    }

  if (fabs(Fy) > FY_MAX)
    {
      Fy = copysign(FY_MAX, Fy);
    }

  tau[0] = Fx;
  tau[1] = Fy;
  tau[2] = L_RUDR * Fy;
}

/**
 * Make sure angle is between [-PI, PI)
 *
 * @todo Naming of function
 */
double normalize_angle(double val)
{
  while (val <= -M_PI)
    val += 2*M_PI;

  while (val > M_PI)
    val -= 2*M_PI;

  return val;
}

/**
 * Makes angle compatible with angle_ref such that the numerical diference is at most PI.
 *
 * @todo Naming of function.
 */
double normalize_angle(double angle, double angle_ref)
{
  double new_angle = 0;
  double diff = angle_ref - angle;

  // Get angle within 2PI of angle_ref
  if (diff > 0)
    {
      new_angle = angle + (diff - fmod(diff, 2*M_PI));
    }
  else
    {
      new_angle = angle + (diff + fmod(-diff, 2*M_PI));
    }

  // Make sure angle is on the closest side of angle_ref
  diff = angle_ref - new_angle;
  if (diff > M_PI)
    {
      new_angle += 2*M_PI;
    }
  else
    {
      new_angle -= 2*M_PI;
    }
  return new_angle;
}
