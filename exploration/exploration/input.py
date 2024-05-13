import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point
from rclpy.node import Node

class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.timer = self.create_timer(1, self.timer_callback)  # Adjust the timer period as needed
        self.x = 0
        self.y = 0
        self.start = False

        self.publisher = self.create_publisher(
            Point,
            '/input',
            10)

    def timer_callback(self):
        user_input = input("Please enter a command: ")  # Get keyboard input
        self.handle_input(user_input)

    def handle_input(self, user_input):
        # Implement your logic based on the input
        # self.get_logger().info(f'You entered: {user_input}')
        x, y = user_input.split(',')
        self.x = float(x)*10
        self.y = float(y)*10
        self.start = True

        self.publish()

        if user_input.lower() == 'quit':
            rclpy.shutdown()

    def publish(self):
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        self.get_logger().info(f'Publishing: {msg.x}, {msg.y}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
