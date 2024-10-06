import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInputSubscriber(Node):
    def __init__(self):
        super().__init__('user_input_subscriber')
        self.subscription = self.create_subscription(
            String,
            'user_input',
            self.user_input_callback,
            10)
        self.subscription 
        self.latest_user_input = None

    def user_input_callback(self, msg):
        self.latest_user_input = msg.data
        self.get_logger().info(f'Received user input: "{self.latest_user_input}"')

def main(args=None):
    rclpy.init(args=args)
    user_input_subscriber = UserInputSubscriber()
    try:
        rclpy.spin(user_input_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        user_input_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
