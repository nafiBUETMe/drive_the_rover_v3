import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import sys

class SerialBridgeNode(Node):
    def __init__(self, port_suffix: str):
        super().__init__('serial_bridge_node')

        self.port = f"/dev/tty{port_suffix}"
        self.baudrate = 9600
        self.serial_conn = None

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        data = f"x {msg.linear.x:.2f} z {msg.angular.z:.2f}"
        self.send_to_serial(data)

    def send_to_serial(self, data: str):
        try:
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)

            self.serial_conn.write((data + '\n').encode('utf-8'))
            self.serial_conn.flush()
            self.get_logger().info(f"📤 Sent: {data}")  # Optional: comment/uncomment
        except Exception:
            self.serial_conn = None  # Forget the connection; try again next time

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("❌ Usage: ros2 run serial_bridge serial_mod USB0|USB1|ACM0")
        rclpy.shutdown()
        return

    port_suffix = sys.argv[1]
    node = SerialBridgeNode(port_suffix)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

