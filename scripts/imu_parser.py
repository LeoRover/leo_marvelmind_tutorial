#!/usr/bin/env python
import rospy
from marvelmind_nav.msg import hedge_imu_fusion as m_imu_fusion
from marvelmind_nav.msg import hedge_imu_raw as m_imu_raw
from sensor_msgs.msg import Imu
import geometry_msgs.msg as geo

import math

class MarvelmindImuParser():
    # Class for parsing messages from marvelmind imu topics  to rosmsg Imu
    
    MG_TO_MS2 = 9.80665 / 1000.0 
    DPS_TO_RPS =  (math.pi / 180.0) * 0.0175

    def __init__(self, fusion_topic_name, raw_topic_name, pub_topic_name, angular_cov, linear_cov, data_frame):
        # Diagonals of covariance matrices for angular velocity and linear acceleration
        self.angular_covariance = angular_cov
        self.linear_covariance  = linear_cov
        
        # Frame for header of the message
        self.frame = data_frame

        # Flags for receiving message from topic
        self.got_fussion_msg = False
        self.got_raw_msg = False

        # Variables to steore the data from callbacks in
        self.orientation = geo.Quaternion()
        self.gyro = geo.Vector3()
        self.acc = geo.Vector3()

        # Sequence for header in imu message
        self.seq = 1

        # Publisher of parsed imu msg
        self.imu_pub = rospy.Publisher(pub_topic_name, Imu, queue_size=1)

        # Subscribers to imu marvelmind topics 
        self.fusion_sub = rospy.Subscriber(fusion_topic_name, m_imu_fusion, self.fusion_callback)
        self.raw_sub = rospy.Subscriber(raw_topic_name, m_imu_raw, self.raw_callback)

    def fusion_callback(self, data):
        self.orientation.x = data.qx
        self.orientation.y = data.qy
        self.orientation.z = data.qz
        self.orientation.w = data.qw

        self.got_fussion_msg = True
        self.imu_publish()

    def raw_callback(self, data):
        # Data on the imu_raw_topic are in:
        # - 1 mg / LSB (mili g per leas significant bit) for accelerometer
        # - 0.0175 dsp / LSB (degree per second per leas significant bit) for gyroscope
        # We need to convert those to:
        # - m / s^2 for acceleration
        # - rad / s for rotational velocity
        # Therefore we have to constants: MG_TO_MS2 (mili g to m / s^2) and DPS_TO_RPS (degree per second to radians per seconds)
        
        self.acc.x = data.acc_x * self.MG_TO_MS2
        self.acc.y = data.acc_y * self.MG_TO_MS2
        self.acc.z = data.acc_z * self.MG_TO_MS2
        
        self.gyro.x = data.gyro_x * self.DPS_TO_RPS
        self.gyro.y = data.gyro_y * self.DPS_TO_RPS
        self.gyro.z = data.gyro_z * self.DPS_TO_RPS

        self.got_raw_msg = True
        self.imu_publish()

    def make_imu_msg(self):
        if self.got_fussion_msg and self.got_raw_msg:
            self.got_raw_msg = False
            self.got_fussion_msg = False

            imu_msg = Imu()
            # filling message header
            imu_msg.header.seq = self.seq
            self.seq += 1
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.frame

            # Filling other fields with data
            imu_msg.orientation = self.orientation
            imu_msg.angular_velocity = self.gyro
            imu_msg.linear_acceleration = self.acc

            # Filling diagonals of covariance matices
            for i in range(3):
                imu_msg.angular_velocity_covariance[i*4] = self.angular_covariance[i]
                imu_msg.linear_acceleration_covariance[i*4] = self.linear_covariance[i]
            
            return imu_msg

        else:
            return None

    def imu_publish(self):
        msg = self.make_imu_msg()
        if msg != None:
            self.imu_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("imu_parser")

    imu_fusion_topic = rospy.get_param("~imu_fusion_sub_topic", "hedge_imu_fusion")
    imu_raw_topic = rospy.get_param("~imu_raw_sub_topic", "hedge_imu_raw")
    imu_pub_topic = rospy.get_param("~imu_pub_topic", "parsed_imu")
    frame = rospy.get_param("~rame", "beacon_frame")
    imu_ang_vel_cov = rospy.get_param("~imu_angular_velocity_covariance_diagonal", [0.000001, 0.000001, 0.00001])
    imu_lin_acc_cov = rospy.get_param("~imu_linear_acceleration_covariance_diagonal", [0.001, 0.001, 0.001])
    
    imu_parser = MarvelmindImuParser(imu_fusion_topic, imu_raw_topic, imu_pub_topic, imu_ang_vel_cov, imu_lin_acc_cov, frame)
    
    rospy.spin()    