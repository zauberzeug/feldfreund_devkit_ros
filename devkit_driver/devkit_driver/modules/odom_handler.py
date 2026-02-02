import numpy as np
import rosys
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
from rclpy.node import Node
from rosys.driving import Odometer
from tf2_ros import TransformBroadcaster


class OdomHandler:
    """Handle the odometry."""

    def __init__(self, node: Node, odom: Odometer):
        self.log = node.get_logger()
        self._odom = odom
        self._node = node

        # Read parameters
        node.declare_parameter('twist_stddev', np.zeros(36).tolist())
        twist_stddev = node.get_parameter('twist_stddev')
        twist_cov = np.asarray(np.diag(twist_stddev.value)).reshape(-1)
        self.log.debug('Linear twist covariance ' + str(twist_cov))
        node.declare_parameter('pose_stddev', np.zeros(36).tolist())
        pose_stddev = node.get_parameter('pose_stddev')
        pose_cov = np.asarray(np.diag(pose_stddev.value)).reshape(-1)
        self.log.debug('Linear pose covariance ' + str(pose_cov))
        node.declare_parameter('publish_tf', False)
        self._publish_tf = node.get_parameter('publish_tf').value

        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = 'odom'
        self._odom_msg.child_frame_id = 'base_link'
        self._odom_msg.pose.covariance = pose_cov
        self._odom_msg.twist.covariance = twist_cov

        self._transform = TransformStamped()
        self._transform.header.frame_id = 'odom'
        self._transform.child_frame_id = 'base_link'

        # Publisher
        self._publisher = node.create_publisher(Odometry, 'odom', 10)
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self._node)
        self._odom.PREDICTION_UPDATED.subscribe(self.publish_odom)
        rosys.on_startup(self.publish_odom)

    def publish_odom(self):
        """Publish odometry data to ros."""
        pose = self._odom.prediction
        timestamp_message = self._node.get_clock().now().to_msg()
        quat = Quaternion(axis=[0, 0, 1], angle=pose.yaw)
        if self._publish_tf:
            self._transform.header.stamp = timestamp_message
            self._transform.transform.translation.x = pose.x
            self._transform.transform.translation.y = pose.y
            self._transform.transform.translation.z = 0.0
            self._transform.transform.rotation.x = quat.x
            self._transform.transform.rotation.y = quat.y
            self._transform.transform.rotation.z = quat.z
            self._transform.transform.rotation.w = quat.w
            self._tf_broadcaster.sendTransform(self._transform)
        velocity = self._odom.current_velocity
        if velocity is None:
            self.log.debug('Velocity is None, skipping publish')
            return
        self._odom_msg.header.stamp = timestamp_message
        self._odom_msg.pose.pose.position.x = pose.x
        self._odom_msg.pose.pose.position.y = pose.y
        self._odom_msg.pose.pose.position.z = 0.0
        self._odom_msg.pose.pose.orientation.x = quat.x
        self._odom_msg.pose.pose.orientation.y = quat.y
        self._odom_msg.pose.pose.orientation.z = quat.z
        self._odom_msg.pose.pose.orientation.w = quat.w
        self._odom_msg.twist.twist.linear.x = velocity.linear
        self._odom_msg.twist.twist.angular.z = velocity.angular
        self._publisher.publish(self._odom_msg)
