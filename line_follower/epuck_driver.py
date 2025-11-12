import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        self.leftMotor = self.robot.getDevice('left wheel motor')
        self.rightMotor = self.robot.getDevice('right wheel motor')

        self.leftMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0)

        self.rightMotor.setPosition(float('inf'))
        self.rightMotor.setVelocity(0)

        self.target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node('my_epuck_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        forward_speed = self.target_twist.linear.x
        angular_speed = self.target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        command_motor_left = min(max(command_motor_left, -6.28), 6.28)
        command_motor_right = min(max(command_motor_right, -6.28), 6.28)

        self.leftMotor.setVelocity(command_motor_left)
        self.rightMotor.setVelocity(command_motor_right)