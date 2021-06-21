import numpy as np
from numpy.core.fromnumeric import shape
from numpy.linalg.linalg import norm
from sklearn.neighbors import KDTree

from rosidl_parser.definition import Include

class Imls_icp:
    
    def __init__(self, _r=0.03, _h=0.1, _iter=10):
        self.m_r = _r
        self.m_h = _h
        self.m_iterations = _iter
        self.pre_pointcloud = None
        self.now_pointcloud = None
        self.kdt = None
        self.target_pointcloude_normal = None

    def set_source_pointcloud(self, pcloud):
        self.now_pointcloud = pcloud

    def set_target_pointcloud(self, pcloud):
        self.pre_pointcloud = pcloud
        self.remove_nan_point(self.pre_pointcloud)
        # 构建前一帧点云的KD树
        self.kdt = KDTree(self.pre_pointcloud)

    def remove_nan_point(pointcloud):
        np.delete(pointcloud, np.where(np.isnan(pointcloud)))

    def match(self):
        # 若没有计算法向量，则计算法向量
        if self.have_normals == False:
            # 寻找附近点
            self.target_pointcloude_normal = np.empty(shape=[1, len(self.pre_pointcloud)])
            for point in self.pre_pointcloud:
                dist ,ind = self.kdt.query(point, k=20)
                points = self.pre_pointcloud[ind]
                # TODO: 判断一下points是不是都是非None
                
                # 计算每一个点的normal
                if len(points) > 3:
                    normal = self.compute_normal(points)
                else:
                    normal = [np.inf, np.inf]
                self.target_pointcloud_normal.append(normal)
        result = np.ones(3)
        covariance = np.ones(3)
        for i in range(self.m_iterations):
            # 位姿变换
            in_cloud = np.zeros(shape=(2,len(self.now_pointcloud)))
            for ix in range(len(self.now_pointcloud)):
                origin_pose = np.array([*self.now_pointcloud[ix], 1])
                now_pose = origin_pose * result
                in_cloud[ix] = np.array(now_pose[0], now_pose[1])

        ref_cloud, ref_normal = self.projection_now_to_target(in_cloud)

    def project_now_to_target(self, in_cloud):
        pass


    
    def compute_normal(self, points):
        """
        计算这些点的法向量
        """
        points = points - np.mean(points, axis=0)
        eigvalue, eigvector = np.linalg.eig(points.T@points)
        min_eig_index = np.argmin(eigvalue)
        return eigvector[:, min_eig_index]