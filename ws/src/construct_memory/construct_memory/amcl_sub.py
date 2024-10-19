import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('Received AMCL pose:')
        self.get_logger().info(f'Position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}')
        self.get_logger().info(f'Orientation: x={msg.pose.pose.orientation.x}, y={msg.pose.pose.orientation.y}, z={msg.pose.pose.orientation.z}, w={msg.pose.pose.orientation.w}')

def main(args=None):
    rclpy.init(args=args)
    amcl_subscriber = AmclSubscriber()
    rclpy.spin(amcl_subscriber)
    amcl_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
