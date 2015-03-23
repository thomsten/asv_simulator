#include "ros/ros.h"

#include <Eigen/Dense>

// Message types
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "asv_simulator.h"
#include "asv_simulator_node.h"

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "asv_simulator_node");
  ros::start();

  ROS_INFO("Started ASV Simulator node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string name = ros::names::clean(ros::this_node::getNamespace());
  if (name.empty())
    name = "asv";


  Vessel my_vessel;

  my_vessel.initialize(priv_nh);

  VesselNode my_vessel_node;
  my_vessel_node.tf_name = name;

  tf::TransformBroadcaster tf = tf::TransformBroadcaster();
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("state", 10);
  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, &VesselNode::cmdCallback, &my_vessel_node);



  my_vessel_node.initialize(&tf, &pose_pub, &odom_pub, &cmd_sub, &my_vessel);
  my_vessel_node.start();

  ros::shutdown();
  return 0;
}

VesselNode::VesselNode(): theVessel_(NULL),
                          initialized_(false),
                          tf_(NULL),
                          pose_pub_(NULL),
                          odom_pub_(NULL),
                          cmd_sub_(NULL),
                          u_d_(0.0),
                          psi_d_(0.0),
                          r_d_(0.0) {}

VesselNode::~VesselNode(){
}

void VesselNode::initialize(tf::TransformBroadcaster* tf,
                            ros::Publisher *pose_pub,
                            ros::Publisher *odom_pub,
                            ros::Subscriber *cmd_sub,
                            Vessel *vessel)
{
  if (!initialized_)
    {
      tf_ = tf;
      pose_pub_ = pose_pub;
      odom_pub_ = odom_pub;
      cmd_sub_ = cmd_sub;

      theVessel_ = vessel;
      initialized_ = true;
    }
  else
    {
      ROS_ERROR("Attempted to initialize VesselNode twice. Doing nothing...");
    }
}

void VesselNode::start()
{

  ros::Rate loop_rate(1.0 / theVessel_->getDT());

  while (ros::ok())
    {
      theVessel_->updateSystem(u_d_, psi_d_, r_d_);

      this->publishData();

      ros::spinOnce();
      loop_rate.sleep();
    }
}


void VesselNode::publishData()
{

  static int counter = 0;

  Eigen::Vector3d eta, nu;

  /// @todo This could be done with less overhead
  theVessel_->getState(eta, nu);

  tf::Transform transform;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseStamped pose;

  transform.setOrigin(tf::Vector3(eta[0],eta[1],0));

  tf::Quaternion q;
  q.setRPY(0,0,eta[2]);
  transform.setRotation(q);

  tf_->sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(),
                                          "map",
                                          tf_name));

  odom.header.seq = counter;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id = tf_name;

  odom.pose.pose.position.x = eta[0];
  odom.pose.pose.position.y = eta[1];

  tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

  odom.twist.twist.linear.x = nu[0];
  odom.twist.twist.linear.y = nu[1];
  odom.twist.twist.angular.z = nu[2];


  odom_pub_->publish(odom);

  pose.header = odom.header;
  pose.pose.position = odom.pose.pose.position;
  pose.pose.orientation = odom.pose.pose.orientation;

  pose_pub_->publish(pose);

  ++counter;
}

void VesselNode::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO_ONCE("Received control input!");

  //boost::mutex::scoped_lock lock(twist_mutex_);

  u_d_ = msg->linear.x;
  psi_d_ = msg->angular.y;
  r_d_ = msg->angular.z;
}
