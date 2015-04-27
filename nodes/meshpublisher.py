#! /usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler as euler2quat

from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node('markerstuff')
    pub = rospy.Publisher("colormap", Marker, queue_size=10)

    mk = Marker()
    mk.header.seq = 0
    mk.header.frame_id = 'map'
    mk.header.stamp = rospy.Time.now()

    mk.ns = 'colormap'
    mk.id = 0
    mk.type = Marker.MESH_RESOURCE
    mk.mesh_resource = "package://asv_simulator/meshes/hovik.STL"
    mk.action = Marker.ADD
    mk.mesh_use_embedded_materials = True

    mk.pose = Pose()

    mk.pose.position.x = -496.
    mk.pose.position.y = 174.76
    mk.pose.position.z = -8.
    q = euler2quat(np.pi/2., 0., 0.)

    mk.pose.orientation.x = q[0]
    mk.pose.orientation.y = q[1]
    mk.pose.orientation.z = q[2]
    mk.pose.orientation.w = q[3]

    mk.scale.x = 780.
    mk.scale.y = 780.
    mk.scale.z = 780.

    mk.lifetime = rospy.Duration()
    mk.color.a = 1
    mk.color.r = 0.3
    mk.color.g = 0.6
    mk.color.b = 0.3
    r = rospy.Rate(0.5)



    while not rospy.is_shutdown():
        pub.publish(mk)
        mk.header.seq += 1
        mk.header.stamp = rospy.Time.now()

        r.sleep()

