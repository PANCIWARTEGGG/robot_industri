#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist

from tf2_ros import TransformBroadcaster


class GlobalCoordinateNode(Node):
    def __init__(self) -> None:
        super().__init__('global_coordinate_node')

        self.declare_parameter('wheel_radius', 0.1)              # r (m)
        self.declare_parameter('wheel_separation', 0.45)         # s (m)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('left_joint_name', 'base_left_wheel_joint')
        self.declare_parameter('right_joint_name', 'base_right_wheel_joint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 30.0)             # Hz (untuk timer fallback)

        self.r: float = self.get_parameter('wheel_radius').value
        self.s: float = self.get_parameter('wheel_separation').value
        self.frame_id: str = self.get_parameter('frame_id').value
        self.child_frame_id: str = self.get_parameter('child_frame_id').value
        self.left_joint: str = self.get_parameter('left_joint_name').value
        self.right_joint: str = self.get_parameter('right_joint_name').value
        self.enable_tf_publish: bool = self.get_parameter('publish_tf').value

        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        self.v_b: float = 0.0        # kecepatan linier robot
        self.omega: float = 0.0      # kecepatan sudut robot

        self.phi_dot_l: float = 0.0
        self.phi_dot_r: float = 0.0

        self.v_l: float = 0.0        # kecepatan linier roda kiri
        self.v_r: float = 0.0        # kecepatan linier roda kanan

        self.icr: Optional[float] = None

        self.last_stamp: Optional[Time] = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f' Global Coordinate Node aktif\n'
            f' r = {self.r} m | s = {self.s} m\n'
            f' frame_id = "{self.frame_id}" | child_frame_id = "{self.child_frame_id}"\n'
            f' Subscribe : /joint_states (joint: {self.left_joint}, {self.right_joint})\n'
            f' Publish   : /odom, /tf'
        )

    def joint_state_callback(self, msg: JointState) -> None:
        current_stamp = Time.from_msg(msg.header.stamp)

        if self.last_stamp is None:
            self.last_stamp = current_stamp
            self.get_logger().info('Menerima /joint_states pertama, mulai tracking...')
            return

        dt = (current_stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = current_stamp

        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warn(
                f'dt tidak valid: {dt:.6f} s, skip iterasi',
                throttle_duration_sec=2.0
            )
            return

        try:
            idx_l = msg.name.index(self.left_joint)
            idx_r = msg.name.index(self.right_joint)
        except ValueError:
            self.get_logger().warn(
                f'Joint tidak ditemukan! Cari: {self.left_joint}, {self.right_joint}. '
                f'Yang ada: {list(msg.name)}',
                throttle_duration_sec=2.0
            )
            return

        if len(msg.velocity) > max(idx_l, idx_r):
            self.phi_dot_l = msg.velocity[idx_l]
            self.phi_dot_r = msg.velocity[idx_r]
        else:
            self.get_logger().warn(
                'JointState tidak punya field velocity!',
                throttle_duration_sec=2.0
            )
            return

        self.v_l = self.r * self.phi_dot_l
        self.v_r = self.r * self.phi_dot_r

        self.v_b = (self.v_l + self.v_r) / 2.0
        self.omega = (self.v_r - self.v_l) / self.s

        dx_dt = (self.r * math.cos(self.theta) / 2.0) * (self.phi_dot_l + self.phi_dot_r)
        dy_dt = (self.r * math.sin(self.theta) / 2.0) * (self.phi_dot_l + self.phi_dot_r)

        dtheta_dt = (self.r / self.s) * (self.phi_dot_r - self.phi_dot_l)

        self.x += dx_dt * dt
        self.y += dy_dt * dt
        self.theta += dtheta_dt * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        delta_v = self.v_r - self.v_l
        if abs(delta_v) > 1e-6:
            self.icr = self.s * (self.v_l + self.v_r) / (2.0 * delta_v)
        else:
            self.icr = None 

        self.publish_odom(msg.header.stamp)
        if self.enable_tf_publish:
            self.publish_tf(msg.header.stamp)

        self.get_logger().info(
            f'x={self.x:+.3f} y={self.y:+.3f} θ={math.degrees(self.theta):+6.2f}° | '
            f'V_B={self.v_b:+.3f} m/s ω={self.omega:+.3f} rad/s | '
            f'ICR={self.icr if self.icr is None else round(self.icr, 3)}',
            throttle_duration_sec=1.0
        )

    def publish_odom(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = self.v_b
        odom.twist.twist.angular.z = self.omega

        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

    def publish_tf(self, stamp) -> None:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()