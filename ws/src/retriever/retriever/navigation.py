from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from typing import Callable, Tuple
import socket
import json
import select
import time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator    = BasicNavigator()
        self.get_logger().info('BasicNavigator initialized')
        self.initial_pose = PoseStamped()
        self.goal_pose    = PoseStamped()
        self.get_logger().info('Flags and poses initialized')
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.get_logger().info('Subscription to /amcl_pose created')
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active')
        self.timer = self.create_timer(1, self.timer_callback)
        
        self.HOST = 'localhost'
        self.PORT = 65432
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(False)
        
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen()
        
        self.get_logger().info(f'Server listening on {self.HOST}:{self.PORT}')
        
        self.inputs = [self.server_socket]
        self.outputs = []
        
        self.get_logger().info('Timer for navigation checks initialized')
        self.get_logger().info('Navigation node initialization complete')
      
    def timer_callback(self):
        '''
        This function is used to check for incoming connections from the retriever agent.
        If a connection is established, the data is received and processed.
        The data is expected to be in JSON format and contains the goal coordinates and a boolean flag to indicate if navigation is required.
        '''
        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, 0)

        for s in readable:
            if s is self.server_socket:
                connection, client_address = s.accept()
                connection.setblocking(False)
                self.inputs.append(connection)
                self.get_logger().info(f'Accepted connection from {client_address}')
            else:
                try:
                    data = s.recv(1024)
                    if data:
                        message = data.decode('utf-8')
                        received_data = json.loads(message)
                        x_coord = received_data['x_coord']
                        y_coord = received_data['y_coord']
                        navigate = received_data['navigate']
                        self.get_logger().info(f'Received: x={x_coord}, y={y_coord}, navigate={navigate}')
                        if navigate:
                            navigation_result = self.navigate_to_goal(x_coord, y_coord)
                            if navigation_result:
                                self.get_logger().info('Navigation to goal completed successfully')
                            else:
                                self.get_logger().warn('Navigation to goal failed or was interrupted')
                            
                    else:
                        self.get_logger().info('Closing connection')
                        self.inputs.remove(s)
                        s.close()
                except ConnectionResetError:
                    # Handle the case where the client abruptly closes the connection
                    self.get_logger().warning('Connection reset by peer')
                    self.inputs.remove(s)
                    s.close()
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'JSON decode error: {e}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error: {e}')

        for s in exceptional:
            self.get_logger().error('Handling exceptional condition for socket')
            self.inputs.remove(s)
            s.close()

    def amcl_pose_callback(self, msg):
        '''
        This function is used to update the initial pose of the robot.
        The initial pose is used to set the starting point for navigation.
        '''
        self.get_logger().info('Received new AMCL pose')
        
        self.initial_pose.header = msg.header
        
        self.initial_pose.pose.position.x = msg.pose.pose.position.x
        self.initial_pose.pose.position.y = msg.pose.pose.position.y
        self.initial_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.initial_pose.pose.orientation.z = msg.pose.pose.orientation.z
        
        self.get_logger().debug(f'Orientation: w={self.initial_pose.pose.orientation.w}, z={self.initial_pose.pose.orientation.z}')
        self.get_logger().info(f'Updated initial pose timestamp: {self.initial_pose.header.stamp}')
        self.navigator.setInitialPose(self.initial_pose)
        
        self.get_logger().info('Initial pose updated and set in navigator')
        self.get_logger().info(f'New pose - x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}')

    def navigate_to_goal(self, goal_pose_x, goal_pose_y):
        '''
        This function is used to navigate the robot to the goal location.
        We are using the BasicNavigator class from nav2_simple_commander to navigate the robot.
        '''
        if self.initial_pose is None:
            self.get_logger().error('Initial pose is not set')
            return False
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_pose_x
        self.goal_pose.pose.position.y = goal_pose_y
        self.goal_pose.pose.orientation.w = self.initial_pose.pose.orientation.w
        self.goal_pose.pose.orientation.z = self.initial_pose.pose.orientation.z
        
        self.navigator.goToPose(self.goal_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            return False
        else:
            self.get_logger().info('Goal has an invalid return status!')
            return False
        return True

    def cancel_navigation(self):
        '''
        This function is used to cancel the current navigation task.
        '''
        self.navigator.cancelTask()


def main(args=None):
    rclpy.init(args=args)
    nav = Navigation()
    try:
        rclpy.spin(nav) 
    except KeyboardInterrupt:
        nav.get_logger().info('Navigation node stopped cleanly')
    except Exception as e:
        nav.get_logger().error(f'Exception in navigation node: {e}')
    finally:
        nav.navigator.lifecycleShutdown()
        for s in nav.inputs:
            s.close()
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
