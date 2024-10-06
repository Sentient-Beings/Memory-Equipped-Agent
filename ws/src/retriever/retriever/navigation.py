from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


class Navigation:
    def __init__(self):
        rclpy.init()
        # initialize the navigator
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()
        # initialize the goal
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        self.goal_pose.pose.orientation.z = 0.0
        
    def navigate_to_goal(self, goal_pose_x, goal_pose_y):
        self.goal_pose.pose.position.x = goal_pose_x
        self.goal_pose.pose.position.y = goal_pose_y
        self.navigator.goToPose(self.goal_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )                    
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return False
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return False
        else:
            print('Goal has an invalid return status!')
            return False
        self.navigator.lifecycleShutdown()
        return True
    
    def cancel_navigation(self):
        self.navigator.cancelTask()
        

if __name__ == '__main__':
    try:
        nav = Navigation()
        nav.navigate_to_goal(2.0, -0.86)
    finally:
        rclpy.shutdown()
