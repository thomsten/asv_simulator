#!/usr/bin/env python

import rospy
import numpy as np
import tf

import geometry_msgs.msg
import nav_msgs.msg

from tf.transformations import quaternion_from_euler as euler2quat

from utils import normalize_angle

class VesselROS(object):
    """ROS wrapper for Vessel model object"""

    def __init__(self, x0, update_rate, is_main_vessel=False, vesseltype='viknes'):
        self.update_rate = update_rate

        # :TODO: Get the following parameters from the parameter server.
        # Another node is handling visualization.
        self.is_main_vessel = is_main_vessel
        self.vesseltype = vesseltype

        if vesseltype == 'revolt':
            self._scale   = 1.0/20.0
            self._length  = 60.0 * self._scale
            self._breadth = 14.5 * self._scale
            # Vertices of a polygon.
            self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                      (-self._length/2,  self._breadth/2),
                                      ( self._length/3,  self._breadth/2),
                                      ( self._length/2,  0              ),
                                      ( self._length/3, -self._breadth/2)])
        elif vesseltype == 'viknes':
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale
            # Vertices of a polygon.
            self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                      (-self._length/2,  self._breadth/2),
                                      ( self._length/3,  self._breadth/2),
                                      ( self._length/2,  0              ),
                                      ( self._length/3, -self._breadth/2)])
        else:
            print "Error in selection of vessel! You tried: ", vesseltype
            print "Defaulting to: \'viknes\'"
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale

        self.model = VesselModel(x0, self.update_rate, vesseltype)
        self.x = self.model.x
        self.u_d = 3.0
        self.psi_d = np.inf
        self.r_d = 0.0

        # Setup ROS specifics
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._cmd_subscriber = rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, self._cmdvel_callback)
        self._pose_publisher = rospy.Publisher('asvpose', geometry_msgs.msg.PoseStamped)
        self._odom_publisher = rospy.Publisher('odom', nav_msgs.msg.Odometry)

        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = 'map'
        self.odom = nav_msgs.msg.Odometry()
        self.odom.header.frame_id = 'map'
        self.odom.pose.pose = self.pose.pose


    def _cmdvel_callback(self, cmd):
        self.u_d   = cmd.linear.x
        # :TODO: Fix this ugly hack?
        self.psi_d = cmd.angular.y
        self.r_d   = cmd.angular.z

    def _update(self):
        # Update model
        self.model.update(self.u_d, self.psi_d, self.r_d)

        # Transform to quaternions
        quat = euler2quat(0, 0, self.x[2])

        self.pose.pose.position.x = self.x[0]
        self.pose.pose.position.y = self.x[1]
        self.odom.twist.twist.linear.x = self.x[3]
        self.odom.twist.twist.angular.z = self.x[5]

        self.pose.pose.orientation.x = quat[0]
        self.pose.pose.orientation.y = quat[1]
        self.pose.pose.orientation.z = quat[2]
        self.pose.pose.orientation.w = quat[3]

        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.seq += 1

        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.seq += 1


        self._pose_publisher.publish(self.pose)
        self._odom_publisher.publish(self.odom)
        self._tf_broadcaster.sendTransform((self.pose.pose.position.x,
                                            self.pose.pose.position.y,
                                            self.pose.pose.position.z),
                                           (self.pose.pose.orientation.x,
                                            self.pose.pose.orientation.y,
                                            self.pose.pose.orientation.z,
                                            self.pose.pose.orientation.w),
                                           rospy.Time.now(),
                                           "asv",
                                           "map")


    def start_sim(self):
        r = rospy.Rate(1. / self.update_rate)

        while not rospy.is_shutdown():
            self._update()
            r.sleep()

class VesselModel(object):
    """3DOF nonlinear vessel model"""

    def __init__(self, x0, h, vessel_model='viknes'):
        self.x = np.copy(x0)
        self.h = h # Integrator time step

        if vessel_model == 'viknes':
            # Set model parameters
            self.d1u = 16.6
            self.d1v = 9900.0
            self.d1r = 330.0
            self.d2u = 8.25
            self.d2v = 330.0
            self.d2r = 0.0

            self.m   = 3300.0
            self.Iz  = 1320.0

            self.lr  = 4.0
            self.Fxmax = 2310.0
            self.Fymax = 28.8

            self.Kp_p = 0.1
            self.Kp_psi = 5.0
            self.Kd_psi = 1.0
            self.Kp_r   = 8.0

        elif vessel_model == 'hurtigruta':
            # Set model parameters
            self.d1u = 16.6
            self.d1v = 9900.0
            self.d1r = 330.0
            self.d2u = 8.25
            self.d2v = 330.0
            self.d2r = 0.0

            self.m   = 3300.0
            self.Iz  = 1320.0

            self.lr  = 4.0
            self.Fxmax = 2310.0
            self.Fymax = 28.8

            self.Kp_p = 0.1
            self.Kp_psi = 5.0
            self.Kd_psi = 1.0
            self.Kp_r   = 8.0


        # Values other algorithms can use to get information about the model

        # Max yawrate:
        # inertia*r_dot = -(d1r + d2r*|r|)*r + fr_max*lx = 0
        if self.d2r > 0:
            self.est_r_max = 0.5*(-self.d1r + \
                                  np.sqrt(self.d1r**2 + 4*self.d1r*self.Fymax*self.lr)) / d2r
        else:
            self.est_r_max = self.Fymax*self.lr / self.d1r

        # Max yaw acceleration (at r = 0):
        self.est_dr_max = self.Fymax*self.lr / self.Iz

        # Max surge velocity
        # mass*u_dot = -(d1u + d2u*|u|)*u + force_max = 0
        if self.d2u > 0:
            self.est_u_max = 0.5*(-self.d1u + \
                                  np.sqrt(self.d1u**2 + 4*self.d1u*self.Fxmax)) / self.d2u
        else:
            self.est_u_max = self.Fxmax / self.d1u;

        # Min surge velocity (max reverse)
        self.est_u_min = -self.est_u_max;

        # Max surge acceleration
        self.est_du_max = self.Fxmax / self.m
        # Min surge acceleration (max reverse)
        self.est_du_min = -self.est_du_max


    def Cvv(self):
        return np.array([ self.x[4] * self.x[5],
                         -self.x[3] * self.x[5],
                          0                    ])
    def Dvv(self):
        return np.array([self.d2u*self.x[3]*np.abs(self.x[3]) + self.d1u*self.x[3],
                         self.d2v*self.x[4]*np.abs(self.x[4]) + self.d1v*self.x[4],
                         self.d1r*self.x[5]])

    def Tau(self, u_d, psi_d, r_d):
        Fx = (self.d1u + self.d2u*np.abs(self.x[3])) * self.x[3] + \
             (self.x[4]*self.x[5] + self.Kp_p*(u_d - self.x[3])) * self.m

        if psi_d == np.inf:
            Fy = 1 / self.lr * ( (self.d1r + self.d2r*np.abs(self.x[5]))*self.x[5] + \
                                 self.Iz * self.Kp_r*(r_d - self.x[5]))
        else:
            Fy = self.Kp_psi * self.Iz / self.lr * ((psi_d - self.x[2]) - self.Kd_psi*self.x[5])

        if np.abs(Fx) > self.Fxmax:
            Fx = np.sign(Fx)*self.Fxmax # :todo: Not realistic to go full speed reverse?

        if np.abs(Fy) > self.Fymax:
            Fy = np.sign(Fy)*self.Fymax

        return np.array([Fx, Fy, self.lr*Fy])

    def update(self, u_d, psi_d, r_d):

        self.x[2] = normalize_angle(self.x[2], psi_d)

        Rz = np.array([[ np.cos(self.x[2]),-np.sin(self.x[2]), 0],
                       [ np.sin(self.x[2]), np.cos(self.x[2]), 0],
                       [ 0                , 0                , 1]])

        self.x[0:3] += self.h * np.dot(Rz, self.x[3:6])
        self.x[3:6] += self.h * np.dot(np.diag([1/self.m, 1/self.m, 1/self.Iz]),
                                       self.Tau(u_d, psi_d, r_d) - self.Cvv() - self.Dvv())

        while self.x[2] >= np.pi:
            self.x[2] -= 2*np.pi

        while self.x[2] < -np.pi:
            self.x[2] += 2*np.pi

        return self.x


if __name__ == "__main__":
    rospy.init_node("ASV_simulator")

    x0 = np.array([0,0,0, 2.0,0,0])
    update_rate = 0.05

    myASV = VesselROS(x0, update_rate, is_main_vessel=True, vesseltype='viknes')
    myASV.start_sim()
