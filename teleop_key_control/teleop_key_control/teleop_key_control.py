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
        self.base_speed = 0.02;
        # self.key_map = {
        #     "i": (self.base_speed, 0.0),
        #     "k": (-self.base_speed, 0.0),
        #     "j": (0.0, self.base_speed),
        #     "l": (0.0, -self.base_speed),
        #     "s": (0.0, 0.0)
        # }
        self._logger.info("""
we are ready, please press 
  i
j k l
to  control the robot and press s to stop the robot
                          """)

        self.pub_vel_from_keyboard()
    def key_map(self, ch):
        
        maps = {
            "u": (self.base_speed, self.base_speed),
            "o": (self.base_speed, -self.base_speed),
            "i": (self.base_speed, 0.0),
            "k": (-self.base_speed, 0.0),
            "j": (0.0, self.base_speed),
            "l": (0.0, -self.base_speed),
            "s": (0.0, 0.0),
        }
        if ch not in maps.keys():
            return None
        return maps[ch]


    def pub_vel_from_keyboard(self):
        msg = Twist()
        try:
            while True:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                if ch == "z":
                    self.base_speed *= 1.2
                    self._logger.info("now speed: {}".format(self.base_speed))
                elif ch == "x":
                    self.base_speed *= 0.8
                    self._logger.info("now speed: {}".format(self.base_speed))

                
                value = self.key_map(ch)
                if value:
                    msg.linear.x, msg.angular.z = float(value[0]), float(value[1])
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
