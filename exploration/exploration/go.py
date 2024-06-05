import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from std_msgs.msg import Bool
import numpy as np
from queue import PriorityQueue
import cv2
import heapq

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.subscription_pose = self.create_subscription(
            Pose2D,
            '/occupancy_map/robot_pose',
            self.robot_pose_callback,
            10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_map',
            self.map_callback,
            10)
        self.subscription_input = self.create_subscription(
            Point,
            '/input',
            self.goal_point_callback,
            10)
        self.publisher_path = self.create_publisher(
            Path,
            '/planned_path',
            10)
        self.subscription_update = self.create_subscription(
            Bool,
            '/update_path',
            self.update_callback,
            10)

        self.current_pose = None
        self.current_map = None
        self.goal_point = None
        self.current_path = None
        self.update = True
        self.directions = [(-5, 0), (5, 0), (0, -5), (0, 5)]
        self.dir = [(-1,0), (1, 0), (0, -1), (0, 1)]
        # self.directions = [(-5, 0), (5, 0), (0, -5), (0, 5), (-10, -10), (-10, 10), (10, -10), (10, 10)]

    def update_callback(self, msg):
        self.update = msg.data

    def robot_pose_callback(self, msg):
        self.current_pose = msg

    def goal_point_callback(self, msg):
        self.update = True
        self.goal_point = msg

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp for the message
        path_msg.header.frame_id = "map"  # Coordinate frame in which the path is represented

        grid = np.array(self.current_map.data).reshape((self.current_map.info.height, self.current_map.info.width))

        # Scale the values to 0 - 255 for visualization (0=free, 100=occupied, -1=unknown)
        image = np.zeros_like(grid, dtype=np.uint8)
        image[grid == -1] = 255  # Free spaces to white
        image[grid == 100] = 0  # Occupied spaces to black
        image[grid == 0] = 127  # Unknown spaces to gray

        # start = (int(self.current_pose.x), int(self.current_pose.y))
        # goal = (int(self.goal_point.x), int(self.goal_point.y))

        # Add the goal point
        # cv2.circle(image, ((start[0]), (start[1])), 5, (255, 0, 255), -1)  # Blue circle at goal point
        # cv2.circle(image, ((goal[0]), (goal[1])), 5, (255, 0, 255), -1)  # Blue circle at goal point

        for point in path:
            # self.get_logger().info(f'point = {point}')
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0] * self.current_map.info.resolution
            pose_stamped.pose.position.y = point[1] * self.current_map.info.resolution
            pose_stamped.pose.orientation.w = 1.0  # Assuming no orientation, quaternion (0,0,0,1)
            path_msg.poses.append(pose_stamped)
            # cv2.circle(image, (point[0], point[1]), 5, (0, 255, 0), -1)  # Green circle at path point
            print(f'point = {point[0]}, {point[1]}')

        # flip y axis
        # flipped_image = np.flipud(image)

        # Display the image using OpenCV
        # cv2.imshow("target point", flipped_image)
        # cv2.waitKey(1)

        self.get_logger().info(f'Publishing path with {len(path)} points')
        self.update = False
        self.publisher_path.publish(path_msg)

    def map_callback(self, msg):
        self.current_map = msg
        grid = np.array(self.current_map.data).reshape((self.current_map.info.height, self.current_map.info.width))
        if self.current_pose and self.goal_point:
            # if self.path_is_obstructed():
            #     self.get_logger().info('Path is obstructed, replanning...')
            #     path = self.plan_path_to_target(self.current_pose, self.goal_point, self.current_map)
            #     self.publish_path(path)
            if self.update:
                # self.get_logger().info('Goal point is unknown, replanning...')
                path = self.plan_path_to_target(self.current_pose, self.goal_point, self.current_map)
                self.publish_path(path)

    def path_is_obstructed(self):
        if self.current_path is None:
            return True
        grid = np.array(self.current_map.data).reshape((self.current_map.info.height, self.current_map.info.width))
        for pose in self.current_path:
            if grid[int(pose.pose.position.y), int(pose.pose.position.x)] == 100:  # Assuming 100 is the obstacle value
                return True
        return False

    def find_nearest_free_space(self, grid, start, goal):
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []
        open_set = set([start])  # Add open set to track items in the priority queue

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]

            if int(grid[current[1], current[0]]) == -1:
                return current

            close_set.add(current)
            open_set.remove(current)  # Remove from open set when popped from heap

            for i, j in self.directions:
                cont = False
                neighbor = current[0] + i, current[1] + j
                if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                    if int(grid[neighbor[1]][neighbor[0]]) == 100 or neighbor in close_set:
                        continue
                    # for k,l in self.dir:
                    #     for n in range(1, 11):
                    #         new_x = neighbor[0] + k * n
                    #         new_y = neighbor[1] + l * n
                    #         if 0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0]:
                    #             if int(grid[new_y][new_x]) == 100:
                    #                 cont = True
                    #                 break
                    # if cont:
                    #     continue
                    tentative_g_score = gscore[current] + heuristic(current, neighbor)
                    if neighbor not in open_set or tentative_g_score < gscore.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        if neighbor not in open_set:
                            heapq.heappush(oheap, (fscore[neighbor], neighbor))
                            open_set.add(neighbor)

        return []  # If no path


    def plan_path_to_target(self, start_pose, goal_point, map):
        self.get_logger().info(f'Planning path from {start_pose} to {goal_point}')
        grid = np.array(map.data).reshape((self.current_map.info.height, self.current_map.info.width))
        self.get_logger().info(f'status of target: {grid[int(goal_point.y), int(goal_point.x)]}')
        self.get_logger().info(f'{type(int(grid[int(goal_point.y), int(goal_point.x)]))}')

        start = (int(start_pose.x), int(start_pose.y))
        goal = (int(goal_point.x), int(goal_point.y))

        if int(grid[int(goal_point.y), int(goal_point.x)]) == 100:
            self.get_logger().info('Goal point is an obstacle')
            return []
        elif int(grid[int(goal_point.y), int(goal_point.x)]) == 0:
            self.get_logger().info('Goal point is unknown')
            goal_point = self.find_nearest_free_space(grid, start=goal, goal=start)
            self.get_logger().info(f'New goal (frontier): {goal_point}')
        elif int(grid[int(goal_point.y), int(goal_point.x)]) == -1:
            self.get_logger().info('Goal is free')

        path = self.plan_path(grid, start, goal)
        self.get_logger().info(f'path length = {len(path)}')
        return path

    def plan_path(self, grid, start, goal):
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []
        open_set = set([start])  # Add open set to track items in the priority queue

        heapq.heappush(oheap, (fscore[start], start))

        while oheap:
            current = heapq.heappop(oheap)[1]
            if heuristic(current, goal) <= 10:  # If close enough to goal, return path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path

            close_set.add(current)
            open_set.remove(current)  # Remove from open set when popped from heap
            # print(grid.shape[0], grid.shape[1]) 400 600
            for i, j in self.directions:
                cont = False
                neighbor = current[0] + i, current[1] + j
                if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                    if int(grid[neighbor[1], neighbor[0]]) == 100:
                        self.get_logger().info(f'grid = 100 at {neighbor}')
                    if int(grid[neighbor[1]][neighbor[0]]) == 100 or neighbor in close_set:
                        # self.get_logger().info(f'Obstacle detected at {neighbor}')
                        continue
                    # for k,l in self.dir:
                    #     for n in range(1, 6):
                    #         new_x = neighbor[0] + k * n
                    #         new_y = neighbor[1] + l * n
                    #         if 0 <= new_x < grid.shape[1] and 0 <= new_y < grid.shape[0]:
                    #             if int(grid[new_y][new_x]) == 100:
                    #                 cont = True
                    #                 break
                    #             # self.get_logger().info(f'Checking {neighbor}\'s neighbor, {new_x}, {new_y}')
                    #     if cont:
                    #         break
                    #     else:
                    #         # self.get_logger().info(f'{neighbor}\'s neighbor cleared')
                    #         pass
                    # if cont:
                    #     self.get_logger().info(f"Obstacle detected at {neighbor}'s neighbor")
                    #     continue
                    tentative_g_score = gscore[current] + heuristic(current, neighbor)
                    if neighbor not in open_set or tentative_g_score < gscore.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        if neighbor not in open_set:
                            heapq.heappush(oheap, (fscore[neighbor], neighbor))
                            open_set.add(neighbor)

        return []  # If no path found


def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
