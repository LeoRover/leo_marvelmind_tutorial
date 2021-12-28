# **leo_marvelmind**
This package was created as part of the [Marvelmind Indoor GPS](https://www.leorover.tech/integrations/marvelmind-indoor-gps) integration tutorial for Leo Rover. It provides configuration for navigation for Leo Rover equipped with Marvelmind Starter Set Super-MP-3D.

## **Nodes**
* `hedge_rcv_bin`  
  Publishes position and rotation data of the mobile beacon mounted to the rover on marvelmind custom message types.
  ### **Published topics:**
  * `hedge_imu_raw` (type: [marvelmind_nav/hedge_imu_raw](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_imu_raw.msg))
  * `hedge_imu_fusion` (type: [marvelmind_nav/hedge_imu_fusion](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_imu_fusion.msg))
  * `hedge_pos_ang` (type: [marvelmind_nav/hedge_pos_ang](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_pos_ang.msg))
  
  Better field description [here](https://marvelmind.com/pics/marvelmind_ROS.pdf).<br><br>

* `wheel_odom_parser`  
  Republishes wheel odometry data published by [leo_firmware](https://github.com/LeoRover/core2_firmware) on message type accepted by [robot_localization](http://wiki.ros.org/robot_localization) package.
  ### **Published topics:**
  * `wheel_odom_with_covariance` (type: [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))
  ### **Subscribed topics:**
  * `wheel_odom` (type: [geometry_msgs/TwistStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html))
  ### **Parameters:**
  * `~pub_topic` (`string`) - default: `wheel_odom_with_covariance`
  Name of the published topic.
  * `~wheel_odom_topic` (`string`) - default: `wheel_odom`
  Name of the subscribed topic with odometry data from wheels.
  * `wheel_odom_covariance_diagonal` (`double[]`) - default: `[0.0001, 0.0, 0.0, 0.0, 0.0, 0.001]`   
  Diagonal of the covariance matrix for wheel odometry.<br><br>

* `imu_parser`  
  Republishes IMU data published by `hedge_rcv_bin` on message types accepted by [robot_localization](http://wiki.ros.org/robot_localization) package.
  ### **Published topics:**
  * `parsed_imu` (type: [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))
  ### **Subscribed topics:**
  * `hedge_imu_raw` (type: [marvelmind_nav/hedge_imu_raw](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_imu_raw.msg))
  *  `hedge_imu_fusion` (type: [marvelmind_nav/hedge_imu_fusion](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_imu_fusion.msg))
  ### **Parameters:**
  * `~imu_fusion_sub_topic` (`string`) - default: `hedge_imu_fusion`   
  Name of the subscribed topic with IMU fusion data from mobile beacon.
  * `~imu_raw_sub_topic` (`string`) - default: `hedge_imu_raw`  
  Name of the subscribed topic with raw IMU data from mobile beacon.
  * `~imu_pub_topic` (`string`) - default: `parsed_imu`  
  Name of the published topic.
  * `~frame` (`string`) - default: `beacon_frame`  
  Frame id in header of published `sensor_msgs/Imu` message. Defaults to the name of the frame defined in beacon's URDF file.
  * `~imu_angular_velocity_covariance_diagonal` (`double[]`) - default: `[0.000001, 0.000001, 0.00001]`  
  Diagonal of the covariance matrix for IMU angular velocity.
  * `~imu_linear_acceleration_covariance_diagonal` (`double[]`) - default: `[0.001, 0.001, 0.001]`  
  Diagonal of the covariance matrix for IMU acceleration. <br><br>

*  `pose_parser`
  Republishes position data from `hedge_rcv_bin` node and odometry data from wheels with covariance as an odometry message.
    ### **Published topics:**
     * `parsed_pose` (type: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
    ### **Subscribed topics:**
     * `hedge_pos_ang` (type: [marvelmind_nav/hedge_pos_ang](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/msg/hedge_pos_ang.msg))
     * `wheel_odom_with_covariance` (type: [geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))
    ### **Parameters:**
     * `~pos_sub_topic` (`string`) - default: `hedge_pos_ang`  
     Name of the subscribed topic with position data from mobile beacon.
     * `~wheel_covariance_topic` (`string`) - default: `wheel_odom_with_covariance`  
     Name of the subscribed topic with wheel odometry data with covariance.
     * `~pose_pub_topic` (`string`) - default: `parsed_pose`  
     Name of the published topic.
     * `~frame_id` (`string`) - default: `map`  
     Frame id in header of published `nav_msgs/Odometry` message. Defaults to `map`, as topic published by this node is used in ukf node that provides `map->odom` transform.  
     * `~child_frame` (`string`) - default: `beacon_frame`  
     Id of child frame in header of published `nav_msgs/Odometry` message. Defaults to `beacon_frame`, as message published by the topic from this node is position of beacon in the map frame.
     * `~pose_covariance_diagonal` (`double[]`) - default: `[0.1, 0.1, 0, 0, 0, 0]`  
     Diagonal of the covariance matrix for rover's pose.
     * `~min_linear_speed_covariance` (`double`) - default: `5.0`  
     Value of the covariance matrix diagonal for rotation about Z axis, when the rover has minimal linear speed (0.2 m/s).
     * `~max_linear_speed_covariance` (`double`) - default: `1.0`  
     Value of the covariance matrix diagonal for rotation about Z axis, when the rover has maximum linear speed (0.4 m/s).<br><br>

* `robot_loc_odom`  
  UKF node from [robot_localization](http://wiki.ros.org/robot_localization) package which publishes `odom->base_footprint` tf transform. Uses linear speed in X axis from`wheel_odom_with_covariance` topic and angular speed around Z axis from `parsed_imu` topic.
  ### **Published topics:**
  * `odometry/filtered_odom` (type: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)) <br><br>

*  `robot_loc_map`  
  UKF node from [robot_localization](http://wiki.ros.org/robot_localization) package which  publishes `map->odom` tf transform.
  Uses X and Y coordinates and yaw from `parsed_pose` topic and linear speed in X axis from`wheel_odom_with_covariance` topic and angular speed around Z axis from `parsed_imu` topic.
    ### **Published topics:**
     * `odometry/filtered` (type: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)) <br><br>

## Launch files
* `marvelmind_localization.launch`
  Starts the `hedge_rcv_bin` from [ROS_marvelmind_package](https://bitbucket.org/marvelmind_robotics/ros_marvelmind_package/src/master/), `imu_parser` and `pose_parser` nodes and `robot_loc_odom` and `robot_loc_map` nodes from [robot_localization](http://wiki.ros.org/robot_localization).

