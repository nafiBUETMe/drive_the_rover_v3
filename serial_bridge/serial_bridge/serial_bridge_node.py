import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pyudev
import serial
import threading

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.baudrate = 9600
        self.serial_devices = {}  # port -> serial.Serial
        self.udev_context = pyudev.Context()

        self.get_logger().info("Checking for already connected serial devices...")
        self.list_existing_devices()

        self.get_logger().info("Started monitoring USB serial devices...")
        self.monitor_thread = threading.Thread(target=self.monitor_usb_serial, daemon=True)
        self.monitor_thread.start()

        # Subscriber for servo angles
        self.create_subscription(
            String,
            'servo_angle_topic',
            self.serial_data_callback,
            10
        )

        # Subscriber for cmd_vel
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def list_existing_devices(self):
        for device in self.udev_context.list_devices(subsystem='tty'):
            devnode = device.device_node
            if devnode and (devnode.startswith('/dev/ttyUSB') or devnode.startswith('/dev/ttyACM')):
                self.handle_device_connected(device)

    def monitor_usb_serial(self):
        monitor = pyudev.Monitor.from_netlink(self.udev_context)
        monitor.filter_by('tty')
        monitor.start()

        for device in iter(monitor.poll, None):
            action = device.action
            devnode = device.device_node

            if devnode and (devnode.startswith('/dev/ttyUSB') or devnode.startswith('/dev/ttyACM')):
                if action == 'add':
                    self.get_logger().info(f"🔌 Connected: {devnode}")
                    self.handle_device_connected(device)
                elif action == 'remove':
                    self.get_logger().info(f"❌ Disconnected: {devnode}")
                    self.handle_device_removed(device)

    def handle_device_connected(self, device):
        port = device.device_node
        if port in self.serial_devices:
            return

        parent = device.find_parent(subsystem='usb', device_type='usb_device')
        vendor = parent.get('ID_VENDOR', 'Unknown Vendor') if parent else 'Unknown Vendor'
        product = parent.get('ID_MODEL', 'Unknown Model') if parent else 'Unknown Model'

        try:
            ser = serial.Serial(port, self.baudrate, timeout=0.1)
            self.serial_devices[port] = ser
            self.get_logger().info(f"Opened serial port: {port} → {vendor} - {product}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")

    def handle_device_removed(self, device):
        port = device.device_node
        ser = self.serial_devices.pop(port, None)
        if ser:
            try:
                ser.close()
                self.get_logger().info(f"Closed serial port: {port}")
            except Exception as e:
                self.get_logger().error(f"Error closing serial port {port}: {e}")

    def serial_data_callback(self, msg: String):
        self.send_to_all_devices(f"servo_angle: {msg.data}")

    def cmd_vel_callback(self, msg: Twist):
        data = f"x {msg.linear.x:.2f} z {msg.angular.z:.2f}"
        self.send_to_all_devices(data)

    def send_to_all_devices(self, data: str):
        for port, ser in self.serial_devices.items():
            try:
                ser.write((data + '\n').encode('utf-8'))
                ser.flush()
                self.get_logger().info(f"Sent to {port}: {data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send to {port}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
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

