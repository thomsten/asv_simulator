#!/usr/bin/env python
import numpy as np
import rosbag
from tf.transformations import quaternion_from_euler as euler2quat
import tf

import rospy

from nav_msgs.msg import Odometry
from  geometry_msgs.msg import TransformStamped


def get_path(data):
    path = np.zeros((data2.shape[0], 3))
    lat = data[:,1]*np.pi/180
    lon = data[:,2]*np.pi/180
    sin_lat = np.sin(lat)
    sin_lon = np.sin(lon)
    cos_lat = np.cos(lat)
    cos_lon = np.cos(lon)

    a = 6378137
    f = 1/298.257223563
    b = a*(1 - f)
    e2 = 1 - (b/a)*(b/a);

    Nphi = a / np.sqrt(1 - e2*np.power(sin_lat, 2))

    ecef = np.array([Nphi*cos_lat*cos_lon,
                     Nphi*cos_lat*sin_lon,
                     Nphi*(1 - e2)*sin_lat])

    p_ecef = ecef - ecef[0]

    phiP = np.arctan2(ecef[0,2], np.sqrt(ecef[0,0]**2 + ecef[0,1]**2))

    sin_phiP = np.sin(phiP)
    cos_phiP = np.cos(phiP)

    rot = np.array([[-sin_phiP*cos_lon[0], -sin_phiP*sin_lon[0], cos_phiP],
                    [-sin_lon[0], cos_lon[0], .0],
                    [cos_phiP*cos_lon[0], cos_phiP*sin_lon[0], sin_phiP]])


    p_ned = np.dot(rot, p_ecef).T

    path[:,0] = p_ned[:,0]
    path[:,1] = p_ned[:,1]
    path[:,0] -= path[0,0]
    path[:,1] -= path[0,1]
    path[:,2] = data2[:,4] - np.pi/2

    return path

if __name__ == "__main__":
    rospy.init_node("anode")
    data = np.loadtxt("/home/thomas/ros_dev/src/asv_simulator/nodes/rlog011.csv", delimiter=',')
    mask1 = np.isfinite(data[:,1])
    data2 = data[mask1]

    path = get_path(data2)
    time = data2[:,0]

    time = (time - time[0])*1e-3 # Convert to seconds
    br = tf.TransformBroadcaster()
    odom_pub = rospy.Publisher("asv/state", Odometry, queue_size=10)

    odom = Odometry()
    odom.header.frame_id = "ned"
    odom.child_frame_id = "asv"

    i = 0
    while not rospy.is_shutdown():
        if i < len(time)-1:
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = path[i,0]
            odom.pose.pose.position.y = path[i,1]
            q = euler2quat(0,0,-path[i,2])
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            # print str(odom.pose.pose.position), time[i]
            odom.header.seq = i

            odom_pub.publish(odom)
            br.sendTransform((path[i,0], path[i,1], 0),
                             q,
                             rospy.Time.now(),
                             "asv",
                             "ned")
            br.sendTransform((0,0,0),
                             euler2quat(np.pi,0,np.pi/2),
                             rospy.Time.now(),
                             "ned",
                             "map")
            i += 1
            d = (time[i]-time[i-1])
            #print "Time to sleep: %.2f seconds"%(d)
            rospy.sleep(d)
        else:
            rospy.spin()
