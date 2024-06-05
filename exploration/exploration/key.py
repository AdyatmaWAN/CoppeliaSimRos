#!/usr/bin/env python
import rclpy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to Twist!
------------------------------------------------
Moving around:
   8: forward
   2: backward
   4: strafe left
   6: strafe right
   7: rotate left
   9: rotate right
   5: stop

CTRL-C to quit
"""

moveBindings = {
    '6': (0, 0, 1, -1),     # Forward
    '4': (0, 0, -1, 1),    # Backward
    '2': (0, -1, 0, 0),    # Strafe left
    '8': (0, 1, 0, 0),     # Strafe right
    '7': (0, 0, 0, 1),     # Rotate left
    '9': (0, 0, 0, -1)     # Rotate right
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key == '5':  # Stop command
                x = 0
                y = 0
                z = 0
                th = 0
            elif key == '\x03':  # Ctrl-C
                break
            else:
                continue  # Skip any unknown key presses

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
