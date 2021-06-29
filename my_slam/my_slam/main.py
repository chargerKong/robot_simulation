import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from my_slam.imls_icp import Imls_icp
import numpy as np

class SLAM(Node):

    def __init__(self):
        super().__init__('my_slam')
        self.matcher = Imls_icp()
        self.odom_path_publisher = self.create_publisher(Path, 'path_odom', 10)
        self.imls_path_publisher = self.create_publisher(Path, 'path_imls', 10)
        # 用于发布Path的两条路径
        # 根据里程计画的轨迹
        self.odom_path = Path()
        self.odom_path.header.frame_id = "odom"
        self.odom_path.header.stamp = self.get_clock().now().to_msg()
        # 根据imls-icp匹配方法得到的轨迹
        self.imls_path = Path()
        self.imls_path.header.frame_id = "odom"
        self.imls_path.header.stamp = self.get_clock().now().to_msg()

        self.scan_subscribe = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_subscribe = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.is_fisrt_frame = True
        # self.pre_pointcloud = None
        self.pre_laser_pose = np.zeros(3)


    def scan_callback(self, msg):
        if self.is_fisrt_frame:
            self.is_fisrt_frame = False
            self.pub_scan_path(
                self.pre_laser_pose, self.imls_path_publisher, self.imls_path)
            self.pre_pointcloud = self.convert_scan_to_pointcloud(msg)
            return 
        
        now_pointcloud = self.convert_scan_to_pointcloud(msg)
        self.matcher.set_source_pointcloud(now_pointcloud)
        self.matcher.set_target_pointcloud(self.pre_pointcloud)

        # 通过match现在和上帧的点云，获得delta_pose
        is_match, delta_pose, cov = self.matcher.match()
        print(delta_pose)
        if is_match:
            print("Match Successful: {}, {}, {}".format(delta_pose[0, 2], delta_pose[1, 2], \
                np.arctan2(delta_pose[1, 0], delta_pose[0, 0])))
            last_pose = np.array([
                [np.cos(self.pre_laser_pose[2]), -np.sin(self.pre_laser_pose[2]), self.pre_laser_pose[0]],
                [np.sin(self.pre_laser_pose[2]), np.cos(self.pre_laser_pose[2]), self.pre_laser_pose[1]],
                [0 ,0 ,1]
            ])

            now_pose = last_pose @ delta_pose

            self.pre_laser_pose = np.array([now_pose[0, 2], now_pose[1, 2], np.arctan2(now_pose[1, 0], now_pose[0, 0])])

            self.pub_scan_path(self.pre_laser_pose, self.imls_path_publisher, self.imls_path)
        else:
            self._logger.info("Match failed")
        
        self.pre_pointcloud = now_pointcloud

        

    def odom_callback(self, msg):
        if self.is_fisrt_frame:
            return
        self.pub_odom_path(msg, self.odom_path_publisher, self.odom_path)
        # nowpose = [msg.pose.pose.position.x,
        #         msg.pose.pose.position.y, 
        #         2 * np.math.acos(msg.pose.pose.orientation.w)]
        # self.pub_scan_path(nowpose, self.imls_path_publisher, self.imls_path)


    def pub_scan_path(self, pose, publisher, path):
        """
        发布激光匹配路径消息
        """
        this_pose_stamp = PoseStamped()
        this_pose_stamp.header.stamp = self.get_clock().now().to_msg()
        this_pose_stamp.header.frame_id = 'odom'

        this_pose_stamp.pose.position.x = pose[0]
        this_pose_stamp.pose.position.y = pose[1]
        
        this_pose_stamp.pose.orientation.w = np.cos(pose[2] / 2)
        this_pose_stamp.pose.orientation.z = np.sin(pose[2] / 2)
        
        path.poses.append(this_pose_stamp)
        publisher.publish(path)


    def pub_odom_path(self, msg, publisher, path):
        """
        发布里程计的路径
        """
        this_pose_stamp = PoseStamped()

        this_pose_stamp.header.stamp = self.get_clock().now().to_msg()
        this_pose_stamp.header.frame_id = 'odom'

        this_pose_stamp.pose.orientation.x = msg.pose.pose.orientation.x
        this_pose_stamp.pose.orientation.y = msg.pose.pose.orientation.y
        this_pose_stamp.pose.orientation.z = msg.pose.pose.orientation.z
        this_pose_stamp.pose.orientation.w = msg.pose.pose.orientation.w

        this_pose_stamp.pose.position.x = msg.pose.pose.position.x
        this_pose_stamp.pose.position.y = msg.pose.pose.position.y

        # this_pose_stamp.pose = msg.pose.pose

        path.poses.append(this_pose_stamp)
        publisher.publish(path)
        # self._logger.info("have published a message")

    def convert_scan_to_pointcloud(self, msg):
        """
        把激光消息转换为激光坐标系下的二维点云
        """
        n = len(msg.ranges)
        pcs = np.zeros((n, 2))
        angle = msg.angle_min
        for i in range(n):
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                continue
            angle += msg.angle_increment
            lx = msg.ranges[i] * np.sin(angle)
            ly = msg.ranges[i] * np.cos(angle)
            if not lx or not ly:
                continue
            pcs[i][0] = lx
            pcs[i][1] = ly
            
        return pcs


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SLAM()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
