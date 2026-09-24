import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoverNode(Node):
    def __init__(self):
        super().__init__('mover_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        msg = Twist()
        elapsed_time = time.time() - self.start_time

        if elapsed_time < 4.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju: Sisi Panjang 1')  
            
        elif elapsed_time < 7.14:  # 4.0 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Belok 90 Derajat (1)')
            
        elif elapsed_time < 9.14:  # 7.14 + 2.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju: Sisi Pendek 1')
            
        elif elapsed_time < 12.28: # 9.14 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Belok 90 Derajat (2)')
            
        elif elapsed_time < 16.28: # 12.28 + 4.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju: Sisi Panjang 2')
            
        elif elapsed_time < 19.42: # 16.28 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Belok 90 Derajat (3)')
            
        elif elapsed_time < 21.42: # 19.42 + 2.0
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Maju: Sisi Pendek 2')
            
        elif elapsed_time < 24.56: # 21.42 + 3.14
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Belok 90 Derajat (Orientasi Awal)')
            
        # Selesai
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Lintasan Persegi Panjang Selesai.')
            self.publisher_.publish(msg)
            
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()