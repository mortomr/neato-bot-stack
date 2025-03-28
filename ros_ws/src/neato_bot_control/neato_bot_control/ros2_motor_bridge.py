import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import threading
import time

class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # Serial setup
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        time.sleep(2)  # Allow Arduino to reset

        # ROS setup
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 10)
        self.create_subscription(Int32MultiArray, 'motor_pwm', self.motor_cmd_cb, 10)

        # Timers
        self.create_timer(0.1, self.query_encoders)   # 10 Hz encoder polling
        self.create_timer(0.1, self.watchdog_check)   # 10 Hz watchdog monitor

        # State
        self.last_cmd_time = self.get_clock().now()
        self.lock = threading.Lock()

        # Serial read thread
        self.reader = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader.start()

    def motor_cmd_cb(self, msg):
        try:
            self.send_serial(f'SET_LPWM {msg.data[0]}')
            self.send_serial(f'SET_RPWM {msg.data[1]}')
            self.last_cmd_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"Motor command error: {e}")

    def watchdog_check(self):
        elapsed = self.get_clock().now() - self.last_cmd_time
        if elapsed.nanoseconds * 1e-9 > 0.5:
            self.send_serial("SET_LPWM 0")
            self.send_serial("SET_RPWM 0")

    def query_encoders(self):
        self.send_serial("GET_ENCODERS")

    def send_serial(self, cmd):
        if not cmd.strip():
            return
        with self.lock:
            print(f"→ {cmd}")
            self.serial_port.write((cmd + '\n').encode())

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode().strip()
                if line.startswith("encoder_l:"):
                    parts = line.split()
                    l = int(parts[0].split(":")[1])
                    r = int(parts[1].split(":")[1])
                    msg = Int32MultiArray()
                    msg.data = [l, r]
                    self.encoder_pub.publish(msg)
            except Exception:
                pass

def main():
    rclpy.init()
    node = MotorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
