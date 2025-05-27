""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""


import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from basekit_driver.communication.communication import Communication
from basekit_driver.data.data_odom import DataOdom


class OdomHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._clock = node.get_clock()
        self._comm = comm
        self.current_pose = PoseStamped()

        # Red parameter
        node.declare_parameter('twist_stddev', np.zeros(36).tolist())
        twist_stddev = node.get_parameter('twist_stddev')
        twist_cov = np.asarray(np.diag(twist_stddev.value)).reshape(-1)
        self._logger.debug('Linear twist convariance ' + str(twist_cov))
        node.declare_parameter('pose_stddev', np.zeros(36).tolist())
        pose_stddev = node.get_parameter('pose_stddev')
        pose_cov = np.asarray(np.diag(pose_stddev.value)).reshape(-1)
        self._logger.debug('Linear pose convariance ' + str(pose_cov))
        node.declare_parameter('publish_tf', False)
        self._publish_tf = node.get_parameter('publish_tf').value

        # Publisher
        self._publisher = node.create_publisher(Odometry, 'odom', 10)
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(node)

        self._data: DataOdom = DataOdom(pose_cov, twist_cov, self._logger)

        comm.register_core_observer(self)

    def publish_odom(self):
        """Publish odometry data to ros."""
        self._publisher.publish(self._data.get_odometry())
        if self._publish_tf:
            self._tf_broadcaster.sendTransform(self._data.get_transform_stamped())

    def update(self, data: dict) -> None:
        """Read the data from a list of words."""
        self._logger.debug(str(data))

        self._data.update_data(
            self._clock.now(),
            data['linear_speed'],
            data['angular_speed'])
        self.publish_odom()
