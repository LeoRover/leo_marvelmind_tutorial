#!/usr/bin/env python
from collections import deque
import rospy
from marvelmind_nav.msg import hedge_pos_ang as m_position
import geometry_msgs.msg as geo
import nav_msgs.msg as nav
import math
import copy


class MarvelmindPoseParser:
    # Class for parsing messages from marvelmind topic with position to ros odometry message

    def __init__(
        self,
        pos_sub_topic,
        wheel_odom_sub_topic,
        pub_topic_name,
        frame_id,
        ch_frame,
        pose_cov,
        min_speed_cov,
        max_speed_cov,
    ):
        # Frame for header of the message
        self.frame = frame_id

        # Child frame for the message
        self.child_frame = ch_frame

        # Diagonal of covariance matrix for pose covariance
        self.pose_cov = pose_cov

        # Variables to store the data from callbacks in
        self.position = geo.Point()
        self.wheel_speed = geo.Twist()

        # Queue to store rovers position
        self.position_history = deque()

        # Flag to determine if we have enough positions in the queue
        self.has_history = False

        # Time of start of the node - needed to measure first 0.5 seconds of collecting rover's position
        self.start_time = rospy.Time.now()

        # Position of the rover 0.5 seconds in the past
        self.prev_position = geo.Point()

        # Sequence for header in odom message
        self.odom_seq = 0

        # Range for covariance value for Yaw
        self.min_cov = min_speed_cov
        self.max_cov = max_speed_cov
        if self.min_cov < self.max_cov:
            rospy.logerr(
                "Covariance for min linear speed smaller than covarianve for max linear speed.\n Covariance for min: %f\n Covariance for max: %f",
                self.min_cov,
                self.max_cov,
            )
            return

        # Publisher of parsed message
        self.odom_publisher = rospy.Publisher(
            pub_topic_name, nav.Odometry, queue_size=1
        )

        # Subscribers to topic with wheel odometry and marvelmind topic with pose
        self.pos_sub = rospy.Subscriber(pos_sub_topic, m_position, self.pos_callback)
        self.wheel_sub = rospy.Subscriber(
            wheel_odom_sub_topic,
            geo.TwistWithCovarianceStamped,
            self.wheel_odom_callback,
        )

    def wheel_odom_callback(self, data):
        self.wheel_speed = data.twist.twist

    def pos_callback(self, data):
        self.position_history.append(copy.deepcopy(data))

        if not self.has_history:
            time = rospy.Time.now()
            duration = time - self.start_time

            if duration.to_sec() < 0.5:
                # not enough data
                # self.last_time = time
                self.has_history = False
            else:
                self.has_history = True
                temp = self.position_history.popleft()
                self.prev_position.x = temp.x_m
                self.prev_position.y = temp.y_m
                self.prev_position.z = temp.z_m
        else:
            while True:
                temp_pos = self.position_history.popleft()
                if data.timestamp_ms - temp_pos.timestamp_ms <= 500:
                    self.prev_position.x = temp_pos.x_m
                    self.prev_position.y = temp_pos.y_m
                    self.prev_position.z = temp_pos.z_m
                    break

        self.position.x = data.x_m
        self.position.y = data.y_m
        self.position.z = data.z_m
        self.send_odom()

    def make_pose(self):
        pose = geo.PoseWithCovariance()

        pose.pose.position = self.position

        speed = self.wheel_speed.linear.x

        if self.has_history:
            x_diff = self.position.x - self.prev_position.x
            y_diff = self.position.y - self.prev_position.y

            yaw = math.atan2(y_diff, x_diff)
            if speed < 0:
                yaw += math.pi
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)

        if abs(speed) < 0.2 or not self.has_history:
            self.pose_cov[-1] = 10000
        else:
            self.pose_cov[-1] = self.max_cov - (
                (abs(speed) - 0.2) * (self.max_cov - self.min_cov) / 0.2
            )

        for i in range(6):
            pose.covariance[i * 7] = self.pose_cov[i]

        return pose

    def make_odometry_msg(self):
        msg = nav.Odometry()

        msg.header.seq = self.odom_seq
        self.odom_seq += 1
        msg.header.frame_id = self.frame
        msg.child_frame_id = self.child_frame
        msg.header.stamp = rospy.Time.now()

        msg.pose = self.make_pose()

        return msg

    def send_odom(self):
        msg = self.make_odometry_msg()
        self.odom_publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pose_parser")

    pos_topic = rospy.get_param("~pos_sub_topic", "hedge_pos_ang")
    wheel_odom_topic = rospy.get_param(
        "~wheel_covariance_topic", "wheel_odom_with_covariance"
    )
    pose_pub_topic = rospy.get_param("~pose_pub_topic", "parsed_pose")
    frame = rospy.get_param("~frame_id", "map")
    ch_frame = rospy.get_param("~child_frame", "beacon_frame")
    pose_covariance_diagonal = rospy.get_param(
        "~pose_covariance_diagonal", [0.001, 0.001, 0, 0, 0, 0]
    )
    min_covariance = rospy.get_param("~min_linear_speed_covariance", 0.01)
    max_covariance = rospy.get_param("~max_linear_speed_covariance", 0.001)

    pose_parser = MarvelmindPoseParser(
        pos_topic,
        wheel_odom_topic,
        pose_pub_topic,
        frame,
        ch_frame,
        pose_covariance_diagonal,
        min_covariance,
        max_covariance,
    )
    rospy.spin()
