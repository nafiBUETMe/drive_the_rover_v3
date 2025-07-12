import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import sys

class SerialBridgeArmNode(Node):
    def __init__(self, port_suffix: str):
        super().__init__('serial_bridge_arm_node')

        self.port = f"/dev/tty{port_suffix}"
        self.baudrate = 9600
        self.serial_conn = None

        self.create_subscription(String, '/arm_command', self.arm_command_callback, 10)

    def arm_command_callback(self, msg: String):
        self.send_to_serial(msg.data)

    def send_to_serial(self, data: str):
        try:
            # Reconnect if needed
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)

            # Send data
            self.serial_conn.write((data + '\n').encode('utf-8'))
            self.serial_conn.flush()
            self.get_logger().info(f"📤 Sent to {self.port}: {data}")

        except (serial.SerialException, OSError) as e:
            # Gracefully reset connection if error occurs
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
            self.serial_conn = None
            self.get_logger().warn(f"⚠️ Write failed (device may be disconnected): {e}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("❌ Usage: ros2 run serial_bridge serial_bridge_arm_node USB0|USB1|ACM0|ACM1")
        rclpy.shutdown()
        return

    port_suffix = sys.argv[1]
    node = SerialBridgeArmNode(port_suffix)

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

