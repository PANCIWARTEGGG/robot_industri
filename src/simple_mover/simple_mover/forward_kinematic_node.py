import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time


class ForwardKinematicNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_node')

        # Parameter fisik robot
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('left_joint_name', 'base_left_wheel_joint')
        self.declare_parameter('right_joint_name', 'base_right_wheel_joint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.r = self.get_parameter('wheel_radius').value
        self.s = self.get_parameter('wheel_separation').value
        self.left_joint = self.get_parameter('left_joint_name').value
        self.right_joint = self.get_parameter('right_joint_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # State robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Posisi roda sebelumnya
        self.prev_phi_l = None
        self.prev_phi_r = None

        # Subscriber
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom_custom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'FK Node aktif | r={self.r} m, s={self.s} m'
        )

    def joint_state_callback(self, msg: JointState):
        # 1. Cari index joint di message
        try:
            idx_l = msg.name.index(self.left_joint)
            idx_r = msg.name.index(self.right_joint)
        except ValueError:
            # Kalau joint tidak ada, skip
            return

        phi_l = msg.position[idx_l]
        phi_r = msg.position[idx_r]

        # 2. Pertama kali? Simpan & keluar
        if self.prev_phi_l is None:
            self.prev_phi_l = phi_l
            self.prev_phi_r = phi_r
            return

        # 3. Hitung perubahan sudut
        d_phi_l = phi_l - self.prev_phi_l
        d_phi_r = phi_r - self.prev_phi_r

        # 4. Jarak tempuh roda
        d_l = self.r * d_phi_l
        d_r = self.r * d_phi_r

        # 5. Gerakan robot
        d = (d_l + d_r) / 2.0
        d_theta = (d_r - d_l) / self.s

        # 6. Update pose (midpoint integration)
        self.x += d * math.cos(self.theta + d_theta / 2.0)
        self.y += d * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalisasi theta ke [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 7. Simpan untuk iterasi berikutnya
        self.prev_phi_l = phi_l
        self.prev_phi_r = phi_r

        # 8. Publish
        self.publish_odom(msg.header.stamp)
        self.publish_tf(msg.header.stamp)

    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Konversi theta ke quaternion
        q = self.theta_to_quaternion(self.theta)
        odom.pose.pose.orientation = q

        # Covariance (opsional, tapi bagus untuk Nav2)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # theta

        self.odom_pub.publish(odom)

    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.theta_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)

    def theta_to_quaternion(self, theta):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicNode()
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