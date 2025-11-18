# import ros2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


#Create a node for webots simulator as ros2 as the controller

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')      
   
        # Create subscribers
        self.create_subscription(Range, 'gs0', self.sensor_0_callback, 1)
        self.create_subscription(Range, 'gs1', self.sensor_1_callback, 1)
        self.create_subscription(Range, 'gs2', self.sensor_2_callback, 1)

        #Create Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Initialize state variables
        self.sensor_0 = 0.0
        self.sensor_1 = 0.0
        self.sensor_2 = 0.0

        command_message = Twist()
    
    #def cmd_vel_callback(self, twist):
        #self.target_twist = twist

    def sensor_0_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_0 = msg.range

    def sensor_1_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_1 = msg.range
        
    def sensor_2_callback(self, msg):
        # Store sensor value when new data arrives
        self.sensor_2 = msg.range

        command_message = Twist()

        # Get parameters
        threshold = self.get_parameter('sensor_threshold').get_parameter_value().double_value
        speed = self.get_parameter('base_speed').get_parameter_value().double_value

        command_message.linear.x = speed #set the speed of the robot in the YAML file

        # Get latest sensor readings
        s0 = self.sensor_0
        s1 = self.sensor_1
        s2 = self.sensor_2
        #speed = self.target_twist.linear.x

        # Apply line-following logic
        line_right = s0 < threshold
        line_center = s1 < threshold
        line_left = s2 < threshold  

        angular_velocity = 0.0
        # Calculate desired velocities
        print(f'Sensor readings: s0={s0}, s1={s1}, s2={s2}')
        if line_center:
            pass  # angular_velocity = 0.0
        elif line_left and not line_right:
            angular_velocity = 1.0
        elif line_right and not line_left:
            angular_velocity = -1.0
        else:
            angular_velocity = 0.0

        #Check angular_velocity value
        print(f'Angular velocity before publishing: {angular_velocity}')

        # Check if the if statements are correct and logical
        print(f'Line following status: center={line_center}, left={line_left}, right={line_right}')

        print(f'Computed velocities: linear={command_message.linear.x}, angular={angular_velocity}')

        command_message.angular.z = angular_velocity

        # Publish command velocities
        # Publish Twist message
        self.cmd_vel_publisher.publish(command_message)
          
def main(args=None):
    rclpy.init(args=args)
    line = LineFollower()
    rclpy.spin(line)
    line.destroy_node() 
    rclpy.shutdown()
    print('Hi from line_follower_node.')

if __name__ == '__main__':
    main()
