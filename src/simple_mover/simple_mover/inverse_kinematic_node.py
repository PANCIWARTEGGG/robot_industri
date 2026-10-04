import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')

        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.45)

        self.r = self.get_parameter('wheel_radius').value
        self.s = self.get_parameter('wheel_separation').value

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        self.left_publisher  = self.create_publisher(Float64, '/left_wheel/command',  10)
        self.right_publisher = self.create_publisher(Float64, '/right_wheel/command', 10)

        self.get_logger().info(
            f'Node IK Aktif | r = {self.r} m, s = {self.s} m'
        )

    def velocity_callback(self, msg: Twist):
        v_b = msg.linear.x
        omega = msg.angular.z

        phi_dot_l = (v_b - (omega * self.s / 2.0)) / self.r
        phi_dot_r = (v_b + (omega * self.s / 2.0)) / self.r

        msg_left = Float64()
        msg_left.data = float(phi_dot_l)
        msg_right = Float64()
        msg_right.data = float(phi_dot_r)

        self.left_publisher.publish(msg_left)
        self.right_publisher.publish(msg_right)

        self.get_logger().info(
            f'v={v_b:.2f} ω={omega:.2f} → L={phi_dot_l:.2f}, R={phi_dot_r:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()