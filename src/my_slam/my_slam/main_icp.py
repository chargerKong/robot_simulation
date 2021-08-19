from inspect import CO_NESTED
from numpy.core.fromnumeric import resize
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sklearn.neighbors import KDTree
import json
# from my_slam.main_icp import ICP
import math
import numpy as np

class ICP():
    def __init__(self) -> None:
        self.iters = 10

    # def set_source_pointcloud(self, pcloud):
    #     self.now_pointcloud = pcloud

    # def set_target_pointcloud(self, pcloud):
    #     self.pre_pointcloud = pcloud
    #     self.kdt = KDTree(self.pre_pointcloud)
    
    def trans_to_isometry2d(self, R, t):
        res = np.eye(3)
        res[:2, :2] = R
        res[:2, 2] = t.squeeze()
        return res

    def find_correspend_point(self, y_data_pre, y_data_now):
        # 寻找对应点, 建立上一帧数据的KD树，循环下一帧的点
        # 寻找距离此点最近的在上一帧中的数据，保存到data_from_pre
        kdt = KDTree(y_data_now)
        data_from_now = []
        delete_idx = []
        for point_idx in range(y_data_pre.shape[0]):
            point = y_data_pre[point_idx:point_idx+1]
            dist, idx = kdt.query(point, k=1)
            if dist > 0.5:
                delete_idx.append(point_idx)
                continue
            data_from_now.append(y_data_now[idx.squeeze()])
        if delete_idx:
            delete_idx = np.array(delete_idx)
            y_data_pre = np.delete(y_data_pre, delete_idx, axis=0)
        return y_data_pre, data_from_now
        
    
    def icpp(self, y_data_pre: np.array, y_data_now: np.array):
        """
        y_data_pre: shape=(n, 2)
        y_data_now: the same snow_tmhape of y_data_pre = (n, 2)
        
        return: R and t. The transformation from pre to now
        """
        total_R = np.eye(3)
        
        for j in range(50):
            pre_tm, now_tm = self.find_correspend_point(y_data_pre, y_data_now)
            
            pre_t, now_t = pre_tm - np.mean(pre_tm, axis=0), now_tm - np.mean(now_tm, axis=0)
            
            W = np.zeros((2,2))

            for i in range(pre_t.shape[0]):
                W += pre_t[i:i+1].T @ now_t[i:i+1]

            U, sigma, V = np.linalg.svd(W)
            R = V @ U.T
            t = np.mean(now_tm, axis=0) - R@np.mean(pre_tm, axis=0).T
            
            y_data_pre = (R @ pre_tm.T).T + t
            
            total_R = self.trans_to_isometry2d(R, t) @ total_R
            
            angle = (np.arctan2(R[1, 0], R[0, 0])) * 180 / np.pi

            if abs(angle) < 0.0001 and sum(abs(t)) < 0.0001:
                # print("iters:", j)
                # print("delta angle is:{}, delta t is: {}".format(angle, t))
                break
            
        return total_R



class SLAM(Node):

    def __init__(self):
        super().__init__('my_slam')
        self.matcher = ICP()
        self.odom_path_publisher = self.create_publisher(Path, 'path_odom', 10)
        self.imls_path_publisher = self.create_publisher(Path, 'path_icp', 10)
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
        self.scan_data = dict()
        self.count = 0
        self.pre_laser_pose = None

    def scan_callback(self, msg):
        if self.is_fisrt_frame:
            if self.pre_laser_pose is None:
                return
            self.is_fisrt_frame = False
            
            self.pub_scan_path(
                self.pre_laser_pose, self.imls_path_publisher, self.imls_path)
            self.pre_pointcloud = self.convert_scan_to_pointcloud(msg)
            return
        self.scan_data[self.count] = msg.ranges.tolist()
        self.count += 1
        print("data lens:", len(self.scan_data))
        # if self.count >= 20:
        #     print("data write")
        #     with open("rotate_data.txt", "w") as f:
        #         f.write(json.dumps(self.scan_data))
            
        now_pointcloud = self.convert_scan_to_pointcloud(msg)
        # with open("rotation.txt", "a") as f:
        #     f.write("{}".format(now_pointcloud)+"\n")
        # # delta_iso = self.matcher.icpp(self.pre_pointcloud, now_pointcloud)
        # 注意，这里需要求当前点云到之前的点云的变换，不同时刻的点云都会被计算为距离原点的点
        # 一堵墙就会距离原点越来越近，求的是从近（now）到远(pre)的变换。

        delta_iso = self.matcher.icpp(now_pointcloud, self.pre_pointcloud)
        # t_temp = (delta_iso@np.hstack((self.pre_pointcloud, np.ones((self.pre_pointcloud.shape[0], 1)))).T)
        # print( np.mean(t_temp.T[:,:2] - now_pointcloud))

        pre_iso = np.array([[np.cos(self.pre_laser_pose[2]), -np.sin(self.pre_laser_pose[2]), self.pre_laser_pose[0]],
                            [np.sin(self.pre_laser_pose[2]), np.cos(self.pre_laser_pose[2]), self.pre_laser_pose[1]],
                            [0, 0, 1]])
        # # now_iso =  delta_iso @ np.array([self.pre_laser_pose[0], self.pre_laser_pose[1], 1])
        # # print(delta_iso)
        now_iso =  pre_iso @ delta_iso  
        
        # now_pose = delta_pose + self.pre_laser_pose
        now_pose = np.array([now_iso[0, 2], now_iso[1, 2], np.arctan2(now_iso[1, 0], now_iso[0, 0])])
        # print("now pose:", now_pose)
        self.pub_scan_path(
            now_pose, self.imls_path_publisher, self.imls_path
        )
        self.pre_laser_pose = now_pose
        # self._logger.info("now pose: x:{}， y:{}, angle:{}".format(now_pose[0], now_pose[1], now_pose[2]))
        self.pre_pointcloud = now_pointcloud

        
        


    def odom_callback(self, msg):
        if self.is_fisrt_frame:
            self.pre_laser_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 
            2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)])
            return
        self.pub_odom_path(msg, self.odom_path_publisher, self.odom_path)


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
            # 计算单个值的三角函数，math比numpy的快
            lx = msg.ranges[i] * math.sin(angle)
            ly = msg.ranges[i] * math.cos(angle)
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
