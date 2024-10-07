from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from typing import Callable
from . import retriever_agent

# setup the goal position retrieval from the retriever agent
navigation_to_pose = Callable[[], (float, float)]
_navigation_to_pose = navigation_to_pose = lambda: (0.0, 0.0)
class navigation_to_pose:
    def getter_pose_command(getter: navigation_to_pose):
        global _navigation_to_pose
        _navigation_to_pose = getter
    def get_pose_command():
        pose = _navigation_to_pose()
        return pose

set_flag = Callable[[], bool]
_set_flag = set_flag = lambda: False
class set_flag:
    def getter_set_flag(getter: set_flag):
        global _set_flag
        _set_flag = getter
    def get_set_flag():
        flag = _set_flag()
        return flag

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator    = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.goal_pose    = PoseStamped()
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.w = None
        self.z = None
        self.navigator.waitUntilNav2Active()
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        if set_flag.get_set_flag():
            (x, y) = navigation_to_pose.get_pose_command()
            self.navigate_to_goal(x, y)
            set_flag.get_set_flag() = False

    def amcl_pose_callback(self, msg):
        self.initial_pose = msg
        self.w = msg.pose.pose.orientation.w
        self.z = msg.pose.pose.orientation.z
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.setInitialPose(self.initial_pose)
        self.get_logger().info('Initial pose updated from /amcl_pose')

    def navigate_to_goal(self, goal_pose_x, goal_pose_y):
        if self.initial_pose is None:
            self.get_logger().error('Initial pose is not set')
            return False
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = goal_pose_x
        self.goal_pose.pose.position.y = goal_pose_y
        self.goal_pose.pose.orientation.w = self.w
        self.goal_pose.pose.orientation.z = self.z
        
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
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
