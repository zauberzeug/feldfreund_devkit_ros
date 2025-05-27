""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from math import cos, sin

from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Time
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class DataOdom:
    """Implements odometry data."""

    def __init__(self, covariance_pose, covariance_twist, logger):
        self._logger = logger
        self._odom = Odometry()
        self._odom.header.frame_id = 'odom'
        self._odom.child_frame_id = 'base_link'
        self._odom.pose.covariance = covariance_pose
        self._odom.twist.covariance = covariance_twist
        self._last_time = None

    def update_data(self, current_time: Time, linear_speed: float, angular_speed: float):
        """Read new data to odometry."""
        if self._last_time is None:
            self._last_time = current_time
        delta_t: Time = current_time - self._last_time
        self._odom.header.stamp = current_time.to_msg()
        self._odom.twist.twist.linear.x = linear_speed
        self._odom.twist.twist.angular.z = angular_speed
        # self._logger.info('Current twist linear x: ' + str(current_twist.linear.x))
        self._odom.pose.pose = calculate_current_pose(
            self._odom.pose.pose,
            self._odom.twist.twist,
            delta_t.nanoseconds / 1e9)
        self._last_time = current_time

    def get_odometry(self) -> Odometry:
        """Return odometry."""
        return self._odom

    def get_transform_stamped(self) -> TransformStamped:
        """Return odometry as transform stamped."""
        t = TransformStamped()
        t.header = self._odom.header
        t.child_frame_id = self._odom.child_frame_id
        t.transform.translation.x = self._odom.pose.pose.position.x
        t.transform.translation.y = self._odom.pose.pose.position.y
        t.transform.translation.z = self._odom.pose.pose.position.z
        t.transform.rotation.x = self._odom.pose.pose.orientation.x
        t.transform.rotation.y = self._odom.pose.pose.orientation.y
        t.transform.rotation.z = self._odom.pose.pose.orientation.z
        t.transform.rotation.w = self._odom.pose.pose.orientation.w
        # Send the transformation
        return t


def calculate_current_pose(
        last_pose: Pose, current_twist: Twist, delta_t: float) -> Pose:
    """Calculate current pose from last pose and current twist."""
    new_pose = Pose()

    (_, _, last_yaw) = euler_from_quaternion(
        [
            last_pose.orientation.x,
            last_pose.orientation.y,
            last_pose.orientation.z,
            last_pose.orientation.w,
        ]
    )

    new_yaw = (
        last_yaw + current_twist.angular.z * delta_t
    )  # compute yaw from quaternion and add angular speed * delta_t

    new_pose.position.x = last_pose.position.x + \
        current_twist.linear.x * cos(new_yaw) * delta_t

    new_pose.position.y = last_pose.position.y + \
        current_twist.linear.x * sin(new_yaw) * delta_t
    quat_tf = quaternion_from_euler(0, 0, new_yaw)
    new_pose.orientation = Quaternion(
        x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3]
    )

    return new_pose
