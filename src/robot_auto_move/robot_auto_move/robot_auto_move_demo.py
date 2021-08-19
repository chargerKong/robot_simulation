import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class RobotMove(Node):

    def __init__(self):
        super().__init__("RobotMove")
        self.subscribe_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dist_thresh = 0.4
        self.turning_speed = 0.1
        self.forward_speed = 0.05

    def scan_callback(self, msg):
        # print(msg)
        self.leftfront_dist = msg.ranges[180]
        self.front_dist = msg.ranges[90]
        self.rightfront_dist = msg.ranges[0]
        self.move()

    def move(self):
        msg = Twist()
        msg.linear.x = 0.0 
        msg.linear.y = 0.0 
        msg.linear.z = 0.0 
        msg.angular.x = 0.0 
        msg.angular.y = 0.0 
        msg.angular.z = 0.0 
        
        d = self.dist_thresh
 

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed
            self._logger.info("first1")
        elif self.leftfront_dist > d and  self.rightfront_dist < d:
            # 左转
            msg.angular.z = -self.turning_speed
            self._logger.info("first2")

        elif self.leftfront_dist < d  and self.rightfront_dist > d:
            msg.angular.z = self.turning_speed
            self._logger.info("first3")

        else:
            self._logger.info("first4")
            msg.angular.z = -self.turning_speed


        self.publisher_.publish(msg)


def main(args=None):

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    robot_move = RobotMove()

    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(robot_move)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_move.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

