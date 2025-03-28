# /ros_ws/src/neato_bot_control/neato_bot_control/cmd_vel_to_pwm.py ?
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class CmdVelToPwm(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pwm')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/motor_pwm',
            10)

        # Tune these to match your bot
        self.max_linear = 0.3   # m/s
        self.max_angular = 1.0  # rad/s
        self.max_pwm = 255

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Simple differential drive model
        left = linear - angular
        right = linear + angular

        # Scale to PWM
        left_pwm = int(max(-1, min(1, left / self.max_linear)) * self.max_pwm)
        right_pwm = int(max(-1, min(1, right / self.max_linear)) * self.max_pwm)

        pwm_msg = Int32MultiArray()
        pwm_msg.data = [left_pwm, right_pwm]

        self.publisher.publish(pwm_msg)
        self.get_logger().info(f'Published PWM: L={left_pwm} R={right_pwm}')

def main():
    rclpy.init()
    node = CmdVelToPwm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
