import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point, Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Path
# from navigation.srv import SetGoal

import numpy as np

class Navigate(Node):
    def __init__(self):
        super().__init__('navigate')
        self.subscriber_robot_pose = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscriber_goal_point = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_point_callback,
            10)
        self.subscriber_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)
        self.subscriber_path = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)
        self.publisher_update_path = self.create_publisher(
            Bool,
            '/update_path',
            10)


        self.timer = self.create_timer(0.1, self.navigate)

        self.robot_pose = Pose2D()
        self.goal_point = Point()
        self.cmd_vel = Twist()
        self.robot_path_received = False
        self.robot_pose_received = False
        self.goal_point_received = False
        self.cmd_vel_received = False
        self.robot_path = Path()

    def robot_pose_callback(self, msg):
        self.robot_pose = msg
        self.robot_pose_received = True
    
    def goal_point_callback(self, msg):
        self.goal_point = msg
        self.goal_point_received = True

    def path_callback(self, msg):
        # self.goal_point = msg.poses[-1].pose.position
        self.robot_path_received = True
        self.robot_path = msg
        self.get_logger().info(f'Path received: {len(self.robot_path.poses)}')
        # for pose in self.robot_path.poses:
        #     self.get_logger().info(f'Pose: {pose.pose.position.x}, {pose.pose.position.y}')

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_received = True


    def navigate(self):
        # self.get_logger().info('Navigating')
        # self.get_logger().info(f'Current position: {self.robot_pose.x}, {self.robot_pose.y}')
        if self.goal_point_received:
            distance = float('inf')

            dx = self.goal_point.x - self.robot_pose.x
            dy = self.goal_point.y - self.robot_pose.y
            distance = np.sqrt(dx**2 + dy**2)
            goal_angle = np.arctan2(dy, dx)
            theta = goal_angle - self.robot_pose.theta

            while theta > np.pi:
                theta -= 2*np.pi
            while theta < -np.pi:
                theta += 2*np.pi

            cmd_vel = Twist()

            if distance > 0.1:
                cmd_vel.linear.y = np.min([0.2 * distance, 0.2])
                cmd_vel.angular.z = 2.0 * theta
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0

            self.publisher_cmd_vel.publish(cmd_vel)

            return True
        elif self.robot_path_received:
            if len(self.robot_path.poses) > 0:
                goal_point = self.robot_path.poses[-1].pose.position
                distance = float('inf')
                # TODO: position is not synchronized with the map
                grid_x = int((self.robot_pose.x / 0.05) + (20 / 0.05 / 2))
                grid_y = int((self.robot_pose.y / 0.05) + (30 / 0.05 / 2))
                grid_next_x = int((goal_point.x / 0.05) + (20 / 0.05 / 2)) - 200
                grid_next_y = int((goal_point.y / 0.05) + (30 / 0.05 / 2)) - 300
                x = (grid_next_x - (20/0.05/2)) * 0.05
                y = (grid_next_y - (30/0.05/2)) * 0.05
                self.get_logger().info(f'Current position: {grid_x}, {grid_y}')
                self.get_logger().info(f'Current position: {self.robot_pose.x}, {self.robot_pose.y}')
                self.get_logger().info(f'Goal point: {grid_next_x}, {grid_next_y}')
                self.get_logger().info(f'Goal point: {x}, {y}')
                dx = x - self.robot_pose.x
                dy = y - self.robot_pose.y
                distance = np.sqrt(dx**2 + dy**2)
                goal_angle = np.arctan2(dy, dx)
                theta = goal_angle - self.robot_pose.theta

                while theta > np.pi:
                    theta -= 2*np.pi
                while theta < -np.pi:
                    theta += 2*np.pi

                cmd_vel = Twist()

                if distance > 0.1:
                    cmd_vel.linear.y = np.min([0.8 * distance, 0.4])
                    cmd_vel.angular.z = 2.0 * theta
                else:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0

                self.publisher_cmd_vel.publish(cmd_vel)

                if distance <= 0.5:
                    self.get_logger().info(f'Reached goal point: {goal_point.x}, {goal_point.y}')
                    self.robot_path.poses.pop(-1)

                return True

            else:
                self.robot_path_received = False
                bool_msg = Bool()
                bool_msg.data = True
                self.publisher_update_path.publish(bool_msg)

        elif self.cmd_vel_received:
            print(self.cmd_vel)
            if self.cmd_vel.linear.x == 0 and self.cmd_vel.linear.y == 0 and self.cmd_vel.angular.z == 0 and self.cmd_vel.angular.x == 0 and self.cmd_vel.angular.y == 0 and self.cmd_vel.linear.z == 0:
                self.cmd_vel_received = False
            self.publisher_cmd_vel.publish(self.cmd_vel)
            # self.cmd_vel_received = False
            return True

def main(args=None):
    rclpy.init(args=args)
    navigate = Navigate()
    rclpy.spin(navigate)
    navigate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()