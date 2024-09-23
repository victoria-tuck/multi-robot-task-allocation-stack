import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from social_navigation_msgs.msg import PathRequest
from nav_msgs.msg import Path

from nav2_simple_commander.robot_navigator import BasicNavigator

class Planner_Wrapper(Node):
    def __init__(self):
        super().__init__(f'planner_wrapper')

        self.declare_parameter('robots', rclpy.Parameter.Type.STRING_ARRAY)
        self.robots = self.get_parameter('robots').value

        self.clock = self.get_clock()

        self.navigator = BasicNavigator()

        self.timer_period = 0.05

        self.paths = {robot[-1]: None for robot in self.robots}

        self.planner_request_subscriber = self.create_subscription(PathRequest, '/path_request', self.request_callback, 1)
        # # self.init_planner_publisher = self.create_publisher(Bool, f'/planner_init', 1)
        self.path_pub = { robot: self.create_publisher(Path, f'/{robot}/path_return', 10) for robot in self.robots }
        self.publish_timer = self.create_timer(self.timer_period, self.publish_paths)

        self.get_logger().info("Planner wrapper initialized")

    def request_callback(self, msg):
        """
        Request a new plan for an agent from the Navigator upon receiving a new request
        """
        id, current_pose, goal_pose = msg.id, msg.current_pose, msg.goal_pose

        success = False
        count = 0
        while not success and count < 100:
            # self.get_logger().info(f"Finding {id}'s path for the {count} time")
            try:
                self.navigator.setInitialPose(current_pose)
                time.sleep(0.1)
                path = self.navigator.getPath(current_pose, goal_pose)

                assert path is not None
                initial_pose_close_x = abs(path.poses[0].pose.position.x - current_pose.pose.position.x) < 0.05
                initial_pose_close_y = abs(path.poses[0].pose.position.y - current_pose.pose.position.y) < 0.05
                goal_pose_close_x = abs(path.poses[-1].pose.position.x - goal_pose.pose.position.x) < 0.05
                goal_pose_close_y = abs(path.poses[-1].pose.position.y - goal_pose.pose.position.y) < 0.05
                assert initial_pose_close_x and initial_pose_close_y and goal_pose_close_x and goal_pose_close_y
                success = True
            except:

                success = False
            count += 1
        # self.get_logger().info(f"Found {id}'s path after {count - 1} tries")

        if success:
            self.paths[id] = path

    def publish_paths(self):
        """
        Publish most recently calculated paths for all agents
        """
        for key, path in self.paths.items():
            if path is not None:
                self.path_pub[key].publish(path)
                # self.get_logger().info(f"Publishing {key}'s path: {path}")
        


def main(args=None):
    rclpy.init(args=args)
    planner_wrapper = Planner_Wrapper()
    rclpy.spin(planner_wrapper)
    planner_wrapper.destroy_node()
    rclpy.shutdown() 


if __name__ == '__main__':
    main()