# cmd_vel_listener.py
# ROS 2 Node that subscribes to /cmd_vel and converts it to motor PWM

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class CmdVelToPWM(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pwm')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Int32MultiArray, '/motor_pwm', 10)

        # Parameters for mapping
        self.max_linear = 0.3   # max linear speed in m/s
        self.max_angular = 1.0  # max angular speed in rad/s
        self.max_pwm = 255      # max PWM value

        self.wheel_base = 0.2   # distance between wheels (meters)
        self.wheel_radius = 0.03  # wheel radius (meters)

    def listener_callback(self, msg):
        v = msg.linear.x     # forward/backward speed
        w = msg.angular.z    # rotation speed

        v_l = v - w * self.wheel_base / 2.0
        v_r = v + w * self.wheel_base / 2.0

        pwm_l = int(max(min((v_l / self.max_linear) * self.max_pwm, self.max_pwm), -self.max_pwm))
        pwm_r = int(max(min((v_r / self.max_linear) * self.max_pwm, self.max_pwm), -self.max_pwm))

        pwm_msg = Int32MultiArray()
        pwm_msg.data = [pwm_l, pwm_r]
        self.publisher.publish(pwm_msg)

        self.get_logger().info(f"/cmd_vel: v={v:.2f}, w={w:.2f} -> PWM L={pwm_l}, R={pwm_r}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
