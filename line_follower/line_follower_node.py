# import ros2 libraries
import rclpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#Robots Constant Attributes
maxSpeed = 6.28
halfDistanceBetweenWheels = 0.045
wheelRadius = 0.025

#Create a node for webots simulator as ros2 as the controller

class MyEpuckDriver():
    def __init__(self, webots_node, properties):
        super().__init__('my_epuck_driver')
        self.robot = webots_node.robot

        #set Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Declare parameters
        self.declare_parameter('sensor_threshold', 600)
        self.declare_parameter('base_speed_ratio', 0.5)
        self.declare_parameter('max_linear_velocity', 0.2)

        # Create subscribers
        self.create_subscription(Float32, 'ground_sensor_0', self.sensor_0_callback, 10)
        self.create_subscription(Float32, 'ground_sensor_1', self.sensor_1_callback, 10)
        self.create_subscription(Float32, 'ground_sensor_7', self.sensor_7_callback, 10)

        # Create publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize state variables
        self.sensor_0 = 0.0
        self.sensor_1 = 0.0
        self.sensor_7 = 0.0

        self.target_twist = Twist()

        # Create control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.target_twist = msg

    def sensor_0_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_0 = msg.data

    def sensor_1_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_1 = msg.data
        
    def sensor_7_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_7 = msg.data

    def control_loop(self):
        threshold = self.get_parameter('sensor_threshold').get_parameter_value().double_value
        speed_ratio = self.get_parameter('base_speed_ratio').get_parameter_value().double_value
        max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        base_speed = speed_ratio * max_linear_velocity
        # Get latest sensor readings
        s0 = self.sensor_0
        s1 = self.sensor_1
        s7 = self.sensor_7
        # Apply line-following logic
        line_right = s0 < threshold
        line_center = s1 < threshold
        line_left = s7 < threshold
        # Calculate desired velocities
        linear_velocity = base_speed
        angular_velocity = 0.0
        if line_center:
            angular_velocity = 0.0
        elif line_left:
            angular_velocity = 1.0
        elif line_right:
            angular_velocity = -1.0
        else:
            angular_velocity = 0.0
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        # Publish command velocities
        # Publish Twist message
        self.cmd_vel_publisher.publish(twist_msg)
          
def main():
        rclpy.init()
        node = MyEpuckDriver()
        rclpy.spin(node)
        rclpy.shutdown()
