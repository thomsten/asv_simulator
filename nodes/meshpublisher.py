#! /usr/bin/env python
import rospy

from visualization_msgs.msg import Marker

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
    mk.mesh_resource = "package://asv_simulator/meshes/testmap.stl"
    mk.action = Marker.ADD
    mk.mesh_use_embedded_materials = True

    mk.pose = Pose()

    mk.pose.position.x = 0
    mk.pose.position.y = 0

    mk.pose.orientation.w = 1

    mk.scale.x = 1
    mk.scale.y = 1
    mk.scale.z = 1

    mk.lifetime = rospy.Duration()
    mk.color.a = 1
    mk.color.r = 0.3
    mk.color.g = 0.6
    mk.color.b = 0.3
    r = rospy.Rate(5)



    while not rospy.is_shutdown():
        pub.publish(mk)
        mk.header.seq += 1
        mk.header.stamp = rospy.Time.now()

        r.sleep()

