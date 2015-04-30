#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from tf.transformations import quaternion_from_euler as euler2quat

DEG2RAD = np.pi/180.0
RAD2DEG = 180.0/np.pi
KNOT2MS = 0.5144

class FakeASV(object):
    """Loads log data and publishes GPS/IMU messages to the ros system"""
    def __init__(self, filepath):
        self.time_data = None
        self.gps_fix_data = None
        self.gps_vel_data = None
        self.yaw_data = None
        self.imu_gyro_data = None
        self.imu_acc_data = None
        self.imu_mag_data = None
        self.imu_rpy_data = None

        self.load_data(filepath)

        self.fix_msg = NavSatFix()
        self.vel_msg = TwistStamped()
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        self.fix_pub = rospy.Publisher("/fix", NavSatFix, queue_size=5)
        self.vel_pub = rospy.Publisher("/vel", TwistStamped, queue_size=5)
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=5)
        self.mag_pub = rospy.Publisher("/mag", MagneticField, queue_size=5)

    def load_data(self, filename):

        # Columns to fetch
        cols = (0, 5, 41, 42, 46, 47, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67)
        data = np.loadtxt(filepath, delimiter=",", usecols=cols, skiprows=0)
        
        self.time_data = data[:, 0] * 1.0e-3
        self.time_data -= self.time_data[0]
        self.gps_fix_data = data[:, 1:4]
        self.gps_fix_data[:, 1:] *= 1.0e-6
        self.gps_vel_data = data[:, 4:6]
        self.gps_vel_data[:,0] *= KNOT2MS*1e-3
        self.gps_vel_data[:,1] *= DEG2RAD*1e-3
        self.imu_gyro_data = data[:,  6:9]
        self.imu_acc_data  = data[:,  9:12]
        self.imu_mag_data  = data[:, 12:15]
        self.imu_rpy_data  = data[:, 15:18]*DEG2RAD
        self.imu_rpy_data -= self.imu_rpy_data[0,:]
        self.imu_rpy_data[:,2] += np.pi
        #self.imu_rpy_data[:,2] = (np.pi - self.imu_rpy_data[:,2])
        

    def publish_fix(self, it):
        if it == 0:
            # Initialize message
            self.fix_msg.header.frame_id = ""
            self.fix_msg.position_covariance_type = 0 # Unknown

        self.fix_msg.header.stamp = rospy.Time.now()
        self.fix_msg.header.seq = it

        if self.gps_fix_data[it, 0] < 1:
            self.fix_msg.status.status = -1
        else:
            self.fix_msg.status.status = 0
        
        self.fix_msg.latitude  = self.gps_fix_data[it, 1]
        self.fix_msg.longitude = self.gps_fix_data[it, 2]        

        self.fix_pub.publish(self.fix_msg)

    def publish_vel(self, it):
        if it == 0:
            # Initialize message
            self.vel_msg.header.frame_id = "asv"
            
        self.vel_msg.header.seq = it
        self.vel_msg.header.stamp = rospy.Time.now()

        self.vel_msg.twist.linear.x = self.gps_vel_data[it,0]*np.cos(self.gps_vel_data[it,1])
        self.vel_msg.twist.linear.y = self.gps_vel_data[it,0]*np.sin(self.gps_vel_data[it,1])
        
        self.vel_pub.publish(self.vel_msg)

    def publish_imu(self, it):
        if it == 0:
            # Initialize message
            self.imu_msg.header.frame_id = "asv"

        self.imu_msg.header.stamp        = rospy.Time.now()
        self.imu_msg.header.seq          = it
        self.imu_msg.orientation         = Quaternion(*euler2quat(*self.imu_rpy_data[it]))
        self.imu_msg.angular_velocity    = Vector3(*self.imu_gyro_data[it])
        self.imu_msg.linear_acceleration = Vector3(*self.imu_acc_data[it])
        self.imu_pub.publish(self.imu_msg)

    def publish_mag(self, it):
        if it == 0:
            # Initialize message
            self.mag_msg.header.frame_id = "asv"

        self.mag_msg.header.stamp   = rospy.Time.now()
        self.mag_msg.header.seq     = it
        self.mag_msg.magnetic_field = Vector3(*self.imu_mag_data[it])

        self.mag_pub.publish(self.mag_msg)

    def start(self):
        iterator = 0
        while not rospy.is_shutdown():
            self.publish_fix(iterator)
            self.publish_vel(iterator)
            self.publish_imu(iterator)
            self.publish_mag(iterator)

            iterator += 1
            if iterator < len(self.time_data):
                d = (self.time_data[iterator] - self.time_data[iterator-1])
                rospy.sleep(d)
            else:
                break

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("fake_asv")

    filepath = "/home/thomas/Dropbox/NTNU/master/code/DataSamples/Matlab/RLOG011.TXT"
    rospy.loginfo("Starting fake_asv! Logfile: %s", filepath)

    fake_asv = FakeASV(filepath)

    fake_asv.start()
    
