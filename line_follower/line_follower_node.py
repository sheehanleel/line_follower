#to read messages from node controller
from std_msgs.msg import Float32


# import ros2 librarys
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


#Robots Constant Attributes
maxSpeed = 6.28
halfDistanceBetweenWheels = 0.045
wheelRadius = 0.025

class LineFollowerDriver(Node):
    def __init__(self, webotsNode, properties):
        super().__init__('epuck_driver')

        self.robot = webotsNode.robot

        #Motors

        self.leftMotor = self.robot.GetDevice('left wheel motor')
        self.rightMotor = self.robot.GetDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        #Ground Sensors
        self.gs0 = self.robot.getDevice('g0')
        self.gs1 = self.robot.getDevice('g1')
        self.gs2 = self.robot.getDevice('g2')
        self.gs0.enable(self.timestep)
        self.gs1.enable(self.timestep)
        self.gs2.enable(self.timestep)

        
        
        #Create the Publisher 
        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', 10)

        #Create the Subcribers (ground sensors)
        self.create_subscription(Float32, 'ground_sensor_0', self.sensor_0_callback, 10)
        self.create_subscription(Float32, 'ground_sensor_1', self.sensor_1_callback, 10)
        self.create_subscription(Float32, 'ground_sensor_7', self.sensor_7_callback, 10)

    #Gets the msg from the subcribers
    def sensor_0_callback(self, msg):
        self.sensor0 = msg.data

    def sensor_1_callback(self, msg):
        self.sensor1 = msg.data

    def sensor_7_callback(self, msg):
        self.sensor7 = msg.data
    
        #Ground Sensors
        """
        self.gs = []
        self.gsNames = ['gs0', 'gs1', 'gs2']

        for name in self.gsNames:
            sensor = robot.getDevice(name)
            sensor.enable(self.timestep)
            self.gs.append(sensor)
        """
        
        """
        #Initialize Motors
        self.leftMotor = robot.getDevice('left wheel motor')
        self.rightMotor = robot.getDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        self.target_twist = Twist() 
        
        """

    def lineFollowingLogic(self):
        print("THIS WOKLS") # to check if this shit works

        #Code behaviour from my controller lab 2 but changed the variables to fit the requirements

        # read ground sensors outputs
        line_right = self.sensor0 < 600
        line_center = self.sensor1 < 600
        line_left = self.sensor7 < 600

        print("Ground Sensors Values: ", self.sensor0, self.sensor1, self.sensor7 )
        print("Ground Sensors: ", line_right, line_center, line_left)

        baseSpeed = maxSpeed

        # initialize motor speeds at MAX SPEED
        leftSpeed  = baseSpeed
        rightSpeed = baseSpeed

        #Implement line following behavior
        if line_center:
            # on track: go forward
            leftSpeed  = 1 * baseSpeed
            rightSpeed = 1 * baseSpeed
        elif line_left:
            # line is on the left: turn left
            leftSpeed  = 1 * baseSpeed
            rightSpeed = 0.77 * baseSpeed
        elif line_right:
            # line is on the right: turn right
            leftSpeed  = 0.77 * baseSpeed
            rightSpeed = 1 * baseSpeed
        else:
            # line is lost: stop or slow down
            leftSpeed = 0
            rightSpeed = 0
        
        # Get the data from the publisher and process it
        twist = Twist()
        twist.linear.x = (leftSpeed + rightSpeed) / 2.0 / maxSpeed
        twist.angular.z = (rightSpeed - leftSpeed) / (2 * halfDistanceBetweenWheels * maxSpeed)

        self.cmdVelPub.publish(twist)
    
def main():
        rclpy.init()
        node = LineFollowerDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
