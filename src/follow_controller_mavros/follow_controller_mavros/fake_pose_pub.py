#!/usr/bin/env python3
"""
Publishes a static zero pose at 30 Hz so EKF2 has a valid vision position
source on the bench (no GPS needed). Position stays at (0,0,0), facing east.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakePosePub(Node):

    def __init__(self):
        super().__init__('fake_pose_pub')
        self._pub = self.create_publisher(PoseStamped, '/mavros/mavros/vision_pose/pose', 10)
        self.create_timer(1.0 / 30.0, self._publish)
        self._msg = PoseStamped()
        self._msg.header.frame_id = 'map'
        self._msg.pose.position.x = 0.0
        self._msg.pose.position.y = 0.0
        self._msg.pose.position.z = 0.0
        self._msg.pose.orientation.x = 0.0
        self._msg.pose.orientation.y = 0.0
        self._msg.pose.orientation.z = 0.0
        self._msg.pose.orientation.w = 1.0
        self.get_logger().info('FakePosePub ready — publishing static (0,0,0) at 30 Hz')

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePosePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
