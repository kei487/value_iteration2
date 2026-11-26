#!/usr/bin/env python3
"""Integrate /cmd_vel and publish odom -> base_link TF (and Odometry)."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 50.0)

        self.x_ = float(self.get_parameter('x').value)
        self.y_ = float(self.get_parameter('y').value)
        self.yaw_ = float(self.get_parameter('yaw').value)
        self.odom_frame_ = str(self.get_parameter('odom_frame').value)
        self.base_frame_ = str(self.get_parameter('base_frame').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.vx_ = 0.0
        self.wz_ = 0.0
        self.last_time_ = self.get_clock().now()

        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f'fake_odom ready at ({self.x_:.2f}, {self.y_:.2f}, {self.yaw_:.2f})'
        )

    def _cmd_vel_cb(self, msg: Twist):
        self.vx_ = msg.linear.x
        self.wz_ = msg.angular.z

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time_).nanoseconds * 1e-9
        self.last_time_ = now
        if dt <= 0.0 or dt > 1.0:
            return

        self.yaw_ += self.wz_ * dt
        self.x_ += self.vx_ * math.cos(self.yaw_) * dt
        self.y_ += self.vx_ * math.sin(self.yaw_) * dt

        qz = math.sin(self.yaw_ * 0.5)
        qw = math.cos(self.yaw_ * 0.5)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_
        t.child_frame_id = self.base_frame_
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster_.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_
        odom.child_frame_id = self.base_frame_
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx_
        odom.twist.twist.angular.z = self.wz_
        self.odom_pub_.publish(odom)


def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
