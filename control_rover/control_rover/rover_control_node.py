import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import threading

MOVE_BINDINGS = {
    'w': (0.7, 0.0),
    's': (-0.7, 0.0),
    'a': (0.0, 0.7),
    'd': (0.0, -0.7)
}

def get_key(timeout=0.1):
    """Non-blocking key reader"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        self.get_logger().info('Use W/A/S/D to control. Press Q to quit.')

        self.running = True
        self.key_thread = threading.Thread(target=self.keyboard_loop)
        self.key_thread.start()

        self.timer = self.create_timer(0.1, self.publish_twist)

    def keyboard_loop(self):
        while self.running:
            key = get_key()
            if key:
                key = key.lower()
                if key == 'q':
                    self.get_logger().info("Quit command received.")
                    self.running = False
                    rclpy.shutdown()
                    return
                elif key in MOVE_BINDINGS:
                    linear, angular = MOVE_BINDINGS[key]
                    self.twist.linear.x = linear
                    self.twist.angular.z = angular
                else:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0

    def publish_twist(self):
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()

