import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tty, termios

class RobotPublisher(Node):

    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # timer_period = 0.5  # seconds
        self.key_map = {
            "i": (-0.01, 0.0),
            "k": (0.01, 0.0),
            "j": (0.0, -0.01),
            "l": (0.0, 0.01),
            "s": (0.0, 0.0)
        }
        self._logger.info("""
we are ready, please press 
  i
j k l
to  control the robot and press s to stop the robot
                          """)
        self.pub_vel_from_keyboard()
    
    def pub_vel_from_keyboard(self):
        msg = Twist()
        try:
            while True:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                if ch in self.key_map.keys():
                    msg.linear.x, msg.angular.z = self.key_map[ch]
                    self.publisher_.publish(msg)
                    self._logger.info("publish x:{}， z:{}".format(msg.linear.x, msg.angular.z))
                elif ord(ch) == 0x3:
                    break
        except e:
            print(e)
            # finally:
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.publisher_.publish(msg)
            

        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RobotPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
