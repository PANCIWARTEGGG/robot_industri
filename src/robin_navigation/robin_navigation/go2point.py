import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class GoToPointNode(Node):
    def __init__(self):
        super().__init__('go2point_node')

        # === Parameter ===
        self.declare_parameter('K_linear', 1.0)       # gain jarak
        self.declare_parameter('K_angular', 2.0)      # gain sudut
        self.declare_parameter('V_MAX', 0.5)          # kecepatan linier max (m/s)
        self.declare_parameter('W_MAX', 1.5)          # kecepatan sudut max (rad/s)
        self.declare_parameter('DIST_THRESHOLD', 0.05)  # toleransi jarak (m)
        self.declare_parameter('ANGLE_THRESHOLD', 0.1)  # toleransi sudut (rad)
        self.declare_parameter('ROTATE_FIRST_ANGLE', 0.5)  # putar dulu kalau error > 30°

        self.K_v = self.get_parameter('K_linear').value
        self.K_w = self.get_parameter('K_angular').value
        self.V_MAX = self.get_parameter('V_MAX').value
        self.W_MAX = self.get_parameter('W_MAX').value
        self.DIST_THRESHOLD = self.get_parameter('DIST_THRESHOLD').value
        self.ANGLE_THRESHOLD = self.get_parameter('ANGLE_THRESHOLD').value
        self.ROTATE_FIRST_ANGLE = self.get_parameter('ROTATE_FIRST_ANGLE').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False

        self.target_x = None
        self.target_y = None
        self.goal_reached = False

        self.create_subscription(Odometry, '/odom_custom', self.odom_callback, 10)
        self.create_subscription(Point, '/goal_point', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/input_ik', 10)

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Go-to-Point Node aktif | '
            f'K_v={self.K_v}, K_w={self.K_w}, '
            f'V_MAX={self.V_MAX}, W_MAX={self.W_MAX}'
        )
        self.get_logger().info(
            'Kirim target via: ros2 topic pub /goal_point geometry_msgs/msg/Point '
            '"{x: 1.0, y: 0.5}" --once'
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.odom_received = True

    def goal_callback(self, msg: Point):
        self.target_x = msg.x
        self.target_y = msg.y
        self.goal_reached = False
        self.get_logger().info(
            f'Target baru: ({self.target_x:.2f}, {self.target_y:.2f})'
        )

    def control_loop(self):
        if not self.odom_received or self.target_x is None:
            return

        if self.goal_reached:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.theta

        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if distance < self.DIST_THRESHOLD:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info(
                f'Goal tercapai! Posisi akhir: '
                f'({self.x:.3f}, {self.y:.3f}) | '
                f'Error: {distance*100:.2f} cm'
            )
            return

        v = self.K_v * distance
        w = self.K_w * angle_error

        if abs(angle_error) > self.ROTATE_FIRST_ANGLE:
            v = 0.0
        else:
            v = self.K_v * distance * math.cos(angle_error)

        v = max(min(v, self.V_MAX), -self.V_MAX)
        w = max(min(w, self.W_MAX), -self.W_MAX)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # self.get_logger().info(
        #     f'Target ({self.target_x:.2f}, {self.target_y:.2f}) | '
        #     f'pos ({self.x:.2f}, {self.y:.2f}) | '
        #     f'd={distance:.3f} m | θ_err={math.degrees(angle_error):.1f}° | '
        #     f'v={v:.2f} ω={w:.2f}',
        #     throttle_duration_sec=0.5
        # )

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPointNode()
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