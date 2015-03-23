#include "ros/ros.h"
#include "asv_simulator.h"

// Standard libraries
#include <cmath>
#include <iostream>

// Linear algebra math
#include <Eigen/Dense>


/// Make sure angle is between [-PI, PI)
double normalize_angle(double val);


/// Makes angle compatible with angle_ref such that the numerical diference is at most PI.
double normalize_angle_diff(double angle, double angle_ref);


Vessel::Vessel()
{
  eta = Eigen::Vector3d::Zero();
  nu  = Eigen::Vector3d::Zero();
}

Vessel::~Vessel() {}

void Vessel::initialize(ros::NodeHandle nh)
{
  if (!nh.getParam("mass", M))
    M = 3980.0;
  if (!nh.getParam("inertia", I_z))
    I_z = 19703.0;
  if (!nh.getParam("dt", DT))
    DT = 0.05;

  if (!nh.getParam("X_udot", X_udot))
    X_udot = 0.0;
  if (!nh.getParam("Y_vdot", Y_vdot))
    Y_vdot = 0.0;
  if (!nh.getParam("Y_rdot", Y_rdot))
    Y_rdot = 0.0;
  if (!nh.getParam("N_vdot", N_vdot))
    N_vdot = 0.0;
  if (!nh.getParam("N_rdot", N_rdot))
    N_rdot = 0.0;

  if (!nh.getParam("X_u", X_u))
    X_u = -50.0;
  if (!nh.getParam("Y_v", Y_v))
    Y_v = -200.0;
  if (!nh.getParam("Y_r", Y_r))
    Y_r = 0.0;
  if (!nh.getParam("N_v", N_v))
    N_v = 0.0;
  if (!nh.getParam("N_r", N_r))
    N_r = -1281.0;

  if (!nh.getParam("X_uu", X_uu))
    X_uu = -135.0;
  if (!nh.getParam("Y_vv", Y_vv))
    Y_vv = -2000.0;
  if (!nh.getParam("N_rr", N_rr))
    N_rr = 0.0;
  if (!nh.getParam("X_uuu", X_uuu))
    X_uuu = 0.0;
  if (!nh.getParam("Y_vvv", Y_vvv))
    Y_vvv = 0.0;
  if (!nh.getParam("N_rrr", N_rrr))
    N_rrr = -3224.0;

  Eigen::Matrix3d Mtot;
  Mtot << M - X_udot, 0, 0,
    0, M-Y_vdot, -Y_rdot,
    0, -Y_rdot, I_z-N_rdot;
  Minv = Mtot.inverse();

  if (!nh.getParam("Fx_min", Fx_min))
    Fx_min = -6550.0;
  if (!nh.getParam("Fx_max", Fx_max))
    Fx_max = 13100.0;
  if (!nh.getParam("Fy_min", Fy_min))
    Fy_min = -650.0;
  if (!nh.getParam("Fy_max", Fy_max))
    Fy_max = 650.0;

  if (!nh.getParam("rudder_length", rudder_length))
      rudder_length = 4.0;

  if (!nh.getParam("Kp_u", Kp_u))
    Kp_u = 0.1;
  if (!nh.getParam("Kp_psi", Kp_psi))
    Kp_psi = 5.0;
  if (!nh.getParam("Kd_psi", Kd_psi))
    Kd_psi = 1.0;
  if (!nh.getParam("Kp_r", Kp_r))
    Kp_r = 8.0;

  std::vector<double> initial_state;
  if (nh.getParam("initial_state", initial_state))
    {
      // Check if the vector supplied is the right size. Else default to zeros.
      if (initial_state.size() == 6)
        {
          eta[0] = initial_state[0];
          eta[1] = initial_state[1];
          eta[2] = initial_state[2];
          nu[0]  = initial_state[3];
          nu[1]  = initial_state[4];
          nu[2]  = initial_state[5];
        }
    }
}

double Vessel::getDT()
{
  return DT;
}

void Vessel::printPose()
{
  std::cout << eta << std::endl << std::endl;
}

void Vessel::setState(Eigen::Vector3d eta_new, Eigen::Vector3d nu_new)
{
  eta = eta_new;
  nu = nu_new;
}

void Vessel::getState(Eigen::Vector3d &eta2, Eigen::Vector3d &nu2)
{
  eta2 = eta;
  nu2 = nu;
}

void Vessel::updateSystem(double u_d, double psi_d, double r_d)
{
  // Ensure psi_d is "compatible" with psi
  psi_d = normalize_angle_diff(psi_d, eta[2]);

  Eigen::AngleAxisd rot_z = Eigen::AngleAxisd(eta[2], Eigen::Vector3d::UnitZ());

  // Calculate coriolis and dampening matrices according to Fossen, 2011 or Stenersen, 2014.
  Cvv[0] = (-M*nu[1] + Y_vdot*nu[1] + Y_rdot*nu[2]) * nu[2];
  Cvv[1] = ( M*nu[0] - X_udot*nu[0]) * nu[2];
  Cvv[2] = (( M*nu[1] - Y_vdot*nu[1] - Y_rdot*nu[2] ) * nu[0] +
            ( -M*nu[0] + X_udot*nu[0] ) * nu[1]);

  Dvv[0] = - (X_u + X_uu*fabs(nu[0]) + X_uuu*nu[0]*nu[0]) * nu[0];
  Dvv[1] = - ((Y_v*nu[1] + Y_r*nu[2]) +
              (Y_vv*fabs(nu[1])*nu[1] + Y_vvv*nu[1]*nu[1]*nu[1]));
  Dvv[2] = - ((N_v*nu[1] + N_r*nu[2]) +
              (N_rr*fabs(nu[2])*nu[2] + N_rrr*nu[2]*nu[2]*nu[2]));

  this->updateControlInput(u_d, psi_d, r_d);

  // Integrate system
  eta += DT * (rot_z * nu);
  nu  += DT * (Minv * (tau - Cvv - Dvv));

  // Keep yaw within [-PI,PI)
  eta[2] = normalize_angle(eta[2]);
}


void Vessel::updateControlInput(double u_d, double psi_d, double r_d)
{
  double Fx = Cvv[0] + Dvv[0] + Kp_u*M*(u_d - nu[0]);
  double Fy = 0.0;

  // If psi_d == inf, then use yaw-rate controller
  if (isinf(psi_d))
    {
      Fy = Cvv[1] + Dvv[1] + I_z*Kp_r*(r_d - nu[2]);
      Fy *= 1.0 / rudder_length;
    }
  else
    {
      Fy = (Kp_psi * I_z ) * ((psi_d - eta[2]) - Kd_psi*nu[2]);
      Fy *= 1.0 / rudder_length;
    }

  if (Fx < Fx_min)
    Fx = Fx_min;
  if (Fx > Fx_max)
    Fx = Fx_max;

  if (Fy < Fy_min)
    Fy = Fy_min;
  if (Fy > Fy_max)
    Fy = Fy_max;

  tau[0] = Fx;
  tau[1] = Fy;
  tau[2] = rudder_length * Fy;
}

double normalize_angle(double val)
{

  if (isinf(val))
    return val;

  while (val <= -M_PI)
    val += 2*M_PI;

  while (val > M_PI)
    val -= 2*M_PI;

  return val;
}

double normalize_angle_diff(double angle, double angle_ref)
{
  double new_angle = 0;
  double diff = angle_ref - angle;

  if (isinf(angle) || isinf(angle_ref))
    return angle;

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
  else if (diff < -M_PI)
    {
      new_angle -= 2*M_PI;
    }
  return new_angle;
}
