import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pyudev
import serial
import threading
import sys

class SerialBridgeNodeV3(Node):
    def __init__(self, port_suffix: str):
        super().__init__('serial_bridge_node_v3')

        self.baudrate = 9600
        self.target_port = f"/dev/tty{port_suffix}"
        self.serial_devices = {}  # port -> serial.Serial
        self.udev_context = pyudev.Context()

        self.get_logger().info(f"Waiting for target serial device: {self.target_port}")
        self.list_existing_devices()

        self.get_logger().info("Started monitoring USB serial devices...")
        self.monitor_thread = threading.Thread(target=self.monitor_usb_serial, daemon=True)
        self.monitor_thread.start()

        self.create_subscription(String, 'servo_angle_topic', self.serial_data_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def list_existing_devices(self):
        for device in self.udev_context.list_devices(subsystem='tty'):
            if device.device_node == self.target_port:
                self.handle_device_connected(device)

    def monitor_usb_serial(self):
        monitor = pyudev.Monitor.from_netlink(self.udev_context)
        monitor.filter_by('tty')
        monitor.start()

        for device in iter(monitor.poll, None):
            if device.device_node == self.target_port:
                if device.action == 'add':
                    self.get_logger().info(f"🔌 Connected: {device.device_node}")
                    self.handle_device_connected(device)
                elif device.action == 'remove':
                    self.get_logger().info(f"❌ Disconnected: {device.device_node}")
                    self.handle_device_removed(device)

    def handle_device_connected(self, device):
        port = device.device_node
        if port != self.target_port:
            return

        if port in self.serial_devices:
            return

        try:
            ser = serial.Serial(port, self.baudrate, timeout=0.1)
            self.serial_devices[port] = ser
            self.get_logger().info(f"✅ Opened serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")

    def handle_device_removed(self, device):
        port = device.device_node
        if port != self.target_port:
            return

        ser = self.serial_devices.pop(port, None)
        if ser:
            try:
                ser.close()
                self.get_logger().info(f"🔒 Closed serial port: {port}")
            except Exception as e:
                self.get_logger().error(f"Error closing serial port {port}: {e}")

    def serial_data_callback(self, msg: String):
        self.send_to_target_device(f"servo_angle: {msg.data}")

    def cmd_vel_callback(self, msg: Twist):
        data = f"x {msg.linear.x:.2f} z {msg.angular.z:.2f}"
        self.send_to_target_device(data)

    def send_to_target_device(self, data: str):
        ser = self.serial_devices.get(self.target_port)
        if ser:
            try:
                ser.write((data + '\n').encode('utf-8'))
                ser.flush()
                self.get_logger().info(f"📤 Sent to {self.target_port}: {data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send to {self.target_port}: {e}")
        else:
            self.get_logger().warn(f"Device {self.target_port} not connected.")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("❌ Usage: ros2 run serial_bridge serial_bridge_v3 USB0|USB1|ACM0|ACM1")
        rclpy.shutdown()
        return

    port_suffix = sys.argv[1]  # e.g., "USB0"
    node = SerialBridgeNodeV3(port_suffix)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for ser in node.serial_devices.values():
            ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

