#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

class WheelOdomParser:
    def __init__(self, sub_topic, pub_topic, wheel_odom_cov):
        self.odom_msg = TwistWithCovarianceStamped()
        for i in range(6):
            self.odom_msg.twist.covariance[i*7] = wheel_odom_cov[i]

        self.odom_pub = rospy.Publisher(pub_topic, TwistWithCovarianceStamped, queue_size=5)

        self.odom_sub = rospy.Subscriber(sub_topic, TwistStamped, self.odom_callback)

    def odom_callback(self, data):
        self.odom_msg.header = data.header
        self.odom_msg.twist.twist = data.twist

        self.odom_pub.publish(self.odom_msg)

if __name__ == '__main__':
    rospy.init_node("wheel_odom_parser")
    wheel_odom_topic = rospy.get_param("~pub_topic", "wheel_odom_with_covariance")
    wheel_odom_cov = rospy.get_param("~wheel_odom_covariance_diagonal", [0.0001, 0.0, 0.0, 0.0, 0.0, 0.001])
    sub_topic = rospy.get_param("~wheel_odom_topic", "wheel_odom")

    wheel_odom_parser = WheelOdomParser(sub_topic, wheel_odom_topic, wheel_odom_cov)
    
    rospy.spin()
