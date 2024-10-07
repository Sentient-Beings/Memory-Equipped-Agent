import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from . import retriever_agent

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
        self.retriever_agent = retriever_agent.RetrieverAgent()

    def user_input_callback(self, msg):
        self.latest_user_input = msg.data
        agent_response = self.retriever_agent.execute_graph(self.latest_user_input)
        self.get_logger().info(f'Agent response: "{agent_response}"')

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
