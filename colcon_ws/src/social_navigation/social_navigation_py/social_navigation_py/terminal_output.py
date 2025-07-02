import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np # Use np.linalg.norm(np.array([pos_robot1[0] - pos_robot2[0], pos_robo1[1] - pos_robot2[1]]))
# from multi_robot_msgs.msg import RobotState  # your custom message
from tabulate import tabulate
from social_navigation_msgs.msg import Plan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from copy import deepcopy

class MultiRobotDashboard(Node):

    def __init__(self):
        super().__init__('multi_robot_dashboard')
        # Store data in a dict: { robot_name: data }
        robot_name_labels = [('robot1', 'R1'),
                             ('robot2', 'R2'),
                             ('robot3', 'R3'),
                             ('robot4', 'R4'),
                             ('robot5', 'R5'),
                             ('robot6', 'R6')]
        self.robot_data = {label: {'Path': [], 'Storage': '', 'Next Waypoint': '', 'Status': '', 'Odometry': np.zeros((2,))}
                           for _, label in robot_name_labels}
        # We use a separate callback group for subscriptions
        # to avoid blocking the timer callback.
        self.subscription_group = MutuallyExclusiveCallbackGroup()
        # If you know the list of robots or can discover them, you can subscribe
        # to each. For a dynamic approach, consider an approach using /rosout or
        # topic discovery. For simplicity, let's assume R1, R2, R3 are known:
        def create_plan_sub(name, label):
            return self.create_subscription(
                Plan,
                f'/{name}/remaining_plan',
                self.make_robot_plan_callback(label),
                10,
                callback_group=self.subscription_group
            )
        def create_status_sub(name, label):
            return self.create_subscription(
                String,
                f'/{name}/robot_status',
                self.make_robot_status_callback(label),
                10,
                callback_group=self.subscription_group
            )
        def create_odom_sub(name, label):
            return self.create_subscription(
                Odometry,
                f'/{name}/odom',
                self.make_robot_odom_callback(label),
                10,
                callback_group=self.subscription_group
            )
        self.plan_subscriptions = {name: create_plan_sub(name, label) for name, label in robot_name_labels}
        self.status_subscriptions = {name: create_status_sub(name, label) for name, label in robot_name_labels}
        self.odom_subscriptions = {name: create_odom_sub(name, label) for name, label in robot_name_labels}
        # Timer to refresh the dashboard every 2 seconds
        self.timer = self.create_timer(2.0, self.print_dashboard)    
        
    def make_robot_plan_callback(self, name: str):
        def robot_plan_callback(msg: Plan):
            # Update our dictionary with the latest info
            self.robot_data[name]['Path'] = [f'R{str(room)}' for room in msg.plan]
        return robot_plan_callback
    
    def make_robot_status_callback(self, name: str):
        def robot_status_callback(msg: String):
            self.robot_data[name]['Status'] = msg.data
        return robot_status_callback
    
    def make_robot_odom_callback(self, name: str):
        def robot_status_callback(msg: Odometry):
            position = msg.pose.pose.position
            self.robot_data[name]['Odometry'] = np.array([round(position.x, 2), round (position.y, 2)])
        return robot_status_callback
        
    def print_dashboard(self):
        """""
        Print an ASCII table of the current robot states.
        """
        # If no states yet, skip
        if not self.robot_data:
            self.get_logger().info("No robot states received yet.")
            return        # Build a table
        print('here')
        table_data = []
        headers = ["Robot", "Path (Rooms)", "Storage State", "Next Waypoint", "Status", "Position", "Distances"]
        
        robot_data = deepcopy(self.robot_data)
        for robot_name, data in sorted(robot_data.items()):
            position = data['Odometry']
            robot_to_robot_distances = []
            for robot_name2, data2 in sorted(robot_data.items()):
                if robot_name != robot_name2:
                    position2 = data2['Odometry']
                    distance = np.linalg.norm(np.array([position[0] - position2[0], position[1] - position2[1]]))
                    distance = round(distance, 2)
                    robot_to_robot_distances.append(distance)
            if any (x < 10 for x in robot_to_robot_distances):
                status = "Too close!"
            else:
                status = "Safe"
            data['Status'] = status

            # Convert the tasks list to a string like (P_R_1, P_R_2)
            tasks_str = "(" + ", ".join(data['Path']) + ")"
            table_data.append([
                robot_name,
                tasks_str,
                data['Storage'],
                data['Next Waypoint'],
                data['Status'],
                data['Odometry'],
                robot_to_robot_distances
            ])        # Print the table using tabulate
        table_str = tabulate(table_data, headers=headers, tablefmt="fancy_grid")
        # Clear the terminal first (optional)
        print("\033c", end="")  # ANSI escape to clear screen
        print(table_str)
    
def main(args=None):
    rclpy.init(args=args)
    dashboard_node = MultiRobotDashboard()    
    try:
        rclpy.spin(dashboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
