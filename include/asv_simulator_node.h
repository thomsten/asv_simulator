#ifndef ASV_SIMULATOR_NODE_H
#define ASV_SIMULATOR_NODE_H

#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "boost/thread.hpp"

#include "asv_simulator.h"

class VesselNode
{
 public:
  VesselNode();
  void initialize(tf::TransformBroadcaster* tf,
                  ros::Publisher *pose_pub,
                  ros::Publisher *odom_pub,
                  ros::Subscriber *cmd_sub,
                  Vessel *vessel);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void publishData();
  void start();

  ~VesselNode();

  std::string tf_name;

 private:
  Vessel *theVessel_;

  bool initialized_;

  tf::TransformBroadcaster *tf_;
  ros::Publisher *pose_pub_;
  ros::Publisher *odom_pub_;
  ros::Subscriber *cmd_sub_;

  double u_d_;
  double psi_d_;
  double r_d_;

};


#endif
